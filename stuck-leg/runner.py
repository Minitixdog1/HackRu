"""The live loop shared by live.py and commander.py.

LiveRunner reads telemetry from the ESP32, keeps the heartbeat going, runs the
stuck-leg model, and (unless watch_only) sends unstick / detach on its own.
Call step() in a tight loop, or start() to run it in a background thread.
"""
import collections
import csv
import os
import threading
import time

from common import CLASS_NAMES, CSV_COLS, LEGS, telemetry_to_row
from decide import OnlineDetector, ReleaseController, release_threshold


class LiveRunner:
    def __init__(self, link, bundle=None, rate=50, cost_false=10, cost_stuck=1, n_consec=3,
                 watch_only=False, log_dir=None, on_event=None):
        self.link = link
        self.bundle = bundle                     # None = no model: telemetry + commands only
        self.rate = rate
        self.thr = release_threshold(cost_false, cost_stuck)
        self.n_consec = n_consec
        self.watch_only = watch_only
        self.on_event = on_event or (lambda text: None)
        self.sim = bool(link.info.get("sim"))
        self.label = 0                           # keypress label for real-robot logs
        self.lock = threading.Lock()
        self.events = collections.deque(maxlen=20)
        self.det = OnlineDetector(bundle, rate) if bundle else None
        self.ctl = ReleaseController(threshold=self.thr, n_consec=n_consec)
        self.walking = False
        self.gait_cmd = {"on": False, "freq": 1.5, "turn": 0.0}   # last walk command sent
        self.p_last = None
        self.msg_last = None
        self._n, self._t0, self.hz = 0, time.monotonic(), 0.0
        self._thread = None
        self._stop = threading.Event()
        self.trial = time.strftime("live_%Y%m%d_%H%M%S")
        self._log_file = self._writer = None
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, f"{self.trial}.csv")
            self._log_file = open(path, "w", newline="")
            self._writer = csv.DictWriter(self._log_file, fieldnames=CSV_COLS)
            self._writer.writeheader()
            self.log_path = path

    # ------------------------------------------------------------ commands
    def send(self, cmd):
        c = cmd.get("cmd")
        with self.lock:
            if c == "gait":
                self.gait_cmd = {"on": bool(cmd.get("on", 1)), "freq": float(cmd.get("freq", self.gait_cmd["freq"])),
                                 "turn": float(cmd.get("turn", 0) or 0)}
            elif c == "stop":
                self.gait_cmd["on"] = False
        self.link.send(cmd)

    def event(self, text):
        with self.lock:
            self.events.append((time.strftime("%H:%M:%S"), text))
        self.on_event(text)

    def reset_controller(self):
        with self.lock:
            self.ctl = ReleaseController(threshold=self.thr, n_consec=self.n_consec)

    # ------------------------------------------------------------ the loop
    def step(self):
        """Process at most one telemetry message. Returns it, or None."""
        msg = self.link.read()
        if msg is None:
            if self.link.seconds_since_data() > 2:
                self.event("no data from the robot for 2 s; reconnecting")
                self.link.connect()
            return None

        now = time.monotonic()
        self._n += 1
        if now - self._t0 >= 1.0:
            self.hz, self._n, self._t0 = self._n / (now - self._t0), 0, now

        label = msg.get("jam", -1) + 1 if self.sim else self.label   # the sim robot knows the truth
        row = telemetry_to_row(msg, self.trial, label)
        if self._writer:
            self._writer.writerow(row)

        cmds = []
        with self.lock:
            self.msg_last = msg
            was_walking, self.walking = self.walking, bool(msg.get("gait", 1))
            if not self.walking:
                if was_walking and self.det:               # don't mix standing and walking in a window
                    self.det = OnlineDetector(self.bundle, self.rate)
                self.p_last = None
            elif self.det:
                if not was_walking and self.ctl.state == "HOLD":
                    self.ctl = ReleaseController(threshold=self.thr, n_consec=self.n_consec)
                self.ctl.attached = [bool(a) for a in msg.get("att", [1, 1, 1, 1])]
                p = self.det.add(row)
                if p is not None:
                    self.p_last = p
                    cmds = self.ctl.update(msg["t"] / 1000, p)
            state = self.ctl.state

        for cmd in cmds:
            truth = f", truth {CLASS_NAMES[label]}" if self.sim else ""
            who = "watch-only, not sent" if self.watch_only else "auto"
            self.event(f"{who}: {describe(cmd)} (state {state}{truth})")
            if not self.watch_only:
                self.send(cmd)
        return msg

    def start(self):
        """Run step() in a background thread until close()."""
        def loop():
            while not self._stop.is_set():
                try:
                    self.step()
                except Exception as e:                      # keep the heartbeat alive no matter what
                    self.event(f"runner error: {e!r}")
                    time.sleep(0.05)
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        self.link.close()                                   # sends stop
        if self._log_file:
            self._log_file.close()

    # ------------------------------------------------------------ status
    def snapshot(self):
        """Thread-safe summary of the robot right now."""
        with self.lock:
            msg = self.msg_last or {}
            att = msg.get("att", [1, 1, 1, 1])
            p = self.p_last
            stuck = [LEGS[i] for i in range(4) if att[i] and p is not None and p[i + 1] >= 0.5]
            acting = (LEGS[self.ctl.leg] if self.ctl.state in ("UNSTICK", "RECHECK", "HOLD")
                      and self.ctl.leg is not None else None)
            return {
                "connected": msg != {},
                "walking": self.walking,
                "attached": [LEGS[i] for i in range(4) if att[i]],
                "detached": [LEGS[i] for i in range(4) if not att[i]],
                "stuck_probability": ({LEGS[i]: round(float(p[i + 1]), 2) for i in range(4) if att[i]}
                                      if p is not None else None),
                "looks_stuck": stuck,
                "release_logic_state": self.ctl.state,
                "acting_on_leg": acting,
                "threshold": round(self.thr, 2),
                "model_loaded": self.bundle is not None,
                "gait_command": dict(self.gait_cmd),
                "leg_angles": msg.get("enc", [0, 0, 0, 0]),
                "simulated": self.sim,
                "sim_jammed_leg": (LEGS[msg["jam"]] if self.sim and msg.get("jam", -1) >= 0 else None),
                "messages_per_second": round(self.hz),
                "recent_events": [f"{t} {e}" for t, e in list(self.events)[-5:]],
            }


def describe(cmd):
    c = cmd.get("cmd")
    if c in ("unstick", "detach", "attach"):
        extra = f" (attempt {cmd['attempt']})" if "attempt" in cmd else ""
        return f"{c} {cmd['leg']}{extra}"
    if c == "stop":
        return f"stop: {cmd.get('reason', '')}".rstrip(": ")
    return c
