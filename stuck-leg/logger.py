"""Log robot telemetry to CSV and label it live with keypresses.

Keys while logging:
  1-4  leg FL / FR / RL / RR is stuck now (press again when it's free; two legs can be stuck)
  0    nothing is stuck
  g    start / stop walking (Wi-Fi robot)
  q    stop and save

Each run = one trial = one CSV file in data/. Keep trials short (1-3 min) and
vary them: speed, surface, battery, how the leg gets stuck.

Sources:
  --source fake     built-in fake robot (pressing 1-4 really "jams" that leg)
  --source serial   ESP32 over USB, newline-delimited JSON (--port COM5 or /dev/ttyUSB0)
  --source udp      ESP32 over Wi-Fi (finds it automatically, or --robot-ip 192.168.4.1)
                    If the ESP32 runs with SIM_SENSORS 1, pressing 1-4 jams that leg on it.

Generate a fake dataset with no keyboard at all:
  python logger.py --source fake --auto --trials 60 --duration 60 --fast
(includes runs with one leg or a diagonal pair already detached, and two legs jammed at once)
"""
import argparse
import csv
import json
import os
import random
import sys
import time

from common import CSV_COLS, DIAGONAL_PAIRS, LEGS, telemetry_to_row

HEARTBEAT_S = 0.1   # PC -> robot heartbeat; the ESP32 stops motors if it goes quiet


# ---------------------------------------------------------------- keyboard
class KeyReader:
    """Non-blocking single-key reader that works on Windows, macOS and Linux."""

    def __enter__(self):
        if os.name == "nt":
            import msvcrt
            self._msvcrt = msvcrt
        elif sys.stdin.isatty():
            import termios, tty
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if os.name != "nt" and hasattr(self, "_old"):
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def get(self):
        """Return the key pressed since last call, or None."""
        if os.name == "nt":
            return self._msvcrt.getwch() if self._msvcrt.kbhit() else None
        if not sys.stdin.isatty():
            return None
        import select
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if ready else None


# ---------------------------------------------------------------- sources
class FakeSource:
    def __init__(self, seed, rate_hz, fast, encoders=True):
        from fake_robot import FakeRobot
        self.robot = FakeRobot(seed, encoders=encoders)
        self.dt = 1.0 / rate_hz
        self.fast = fast
        self._next = time.monotonic()

    def read(self):
        if not self.fast:                       # pace at real time
            self._next += self.dt
            time.sleep(max(0.0, self._next - time.monotonic()))
        msg = self.robot.step(self.dt)
        msg["att"] = [int(leg not in self.robot.detached) for leg in LEGS]
        return msg

    def set_jams(self, legs):                   # only the fake robot can be "jammed" by software
        for leg in list(self.robot.stuck):
            if leg not in legs:
                self.robot.free(leg)
        for leg in legs:
            if leg not in self.robot.stuck:
                self.robot.add_stuck(leg, self.robot.rng.choice([1.0, 0.8, 0.5, 0.3]))

    def set_detached(self, legs):
        for leg in legs:
            self.robot.detach(leg)



    def heartbeat(self):
        pass


class SerialSource:
    def __init__(self, port, baud):
        import serial                           # pip install pyserial
        self.ser = serial.Serial(port, baud, timeout=0.5)

    def read(self):
        line = self.ser.readline().decode(errors="ignore").strip()
        try:
            return json.loads(line) if line.startswith("{") else None   # skip debug prints
        except json.JSONDecodeError:
            return None                                                  # half a line after reset

    def heartbeat(self):
        self.ser.write(b'{"cmd":"hb"}\n')


class UdpSource:
    """ESP32 over Wi-Fi, via robot_link.RobotLink (discovery + heartbeat)."""

    def __init__(self, robot_ip=None, walk=True):
        from robot_link import RobotLink
        self.link = RobotLink(robot_ip)
        print("looking for the robot...")
        info = self.link.connect()
        self.sim = bool(info.get("sim"))
        print(f"connected to {self.link.robot[0]} ({'SIMULATED sensors' if self.sim else 'real sensors'})")
        self.walking = walk
        self.link.send({"cmd": "gait", "on": int(walk)})

    def read(self):
        return self.link.read()

    def set_jams(self, legs):                   # only does something on a SIM_SENSORS robot
        if self.sim:
            self.link.send({"cmd": "sim_jam", "leg": None})
            for leg in legs:
                self.link.send({"cmd": "sim_jam", "leg": leg})

    def toggle_gait(self):
        self.walking = not self.walking
        self.link.send({"cmd": "gait", "on": int(self.walking)})

    def heartbeat(self):
        self.link.heartbeat()

    def close(self):
        self.link.close()


# ---------------------------------------------------------------- auto labels (fake only)
class AutoScheduler:
    """Randomly jams legs for 1.5-4 s, then frees them for 3-8 s.

    Each trial also picks which legs are already off for the whole run: none (55%),
    one leg (25%) or a diagonal pair (20%), so the model learns what a detached leg
    looks like. About 1 jam in 4 jams two legs at once.

    The label lags the real change by 0.1-0.35 s, like a person's keypress would,
    so the fake labels are as imperfect as real ones.
    """

    def __init__(self, seed):
        self.rng = random.Random(seed)
        r = self.rng.random()
        if r < 0.55:
            self.detached = set()
        elif r < 0.80:
            self.detached = {self.rng.choice(LEGS)}
        else:
            self.detached = set(self.rng.choice(DIAGONAL_PAIRS))
        self.attached = [leg for leg in LEGS if leg not in self.detached]
        self.legs = set()
        self.until = self.rng.uniform(3, 8)
        self.label_legs = set()
        self.pending = []                 # (time the label catches up, legs)

    def update(self, t):
        """Return (legs physically stuck now, legs the label says are stuck)."""
        if t >= self.until:
            if not self.legs:
                n = 2 if len(self.attached) >= 2 and self.rng.random() < 0.25 else 1
                self.legs = set(self.rng.sample(self.attached, n))
                # Most jams are short; about 1 in 8 is "really stuck" (8-12 s).
                long_jam = self.rng.random() < 0.125
                self.until = t + (self.rng.uniform(8, 12) if long_jam else self.rng.uniform(1.5, 4.0))
            else:
                self.legs = set()
                self.until = t + self.rng.uniform(3.0, 8.0)
            self.pending.append((t + self.rng.uniform(0.1, 0.35), set(self.legs)))
        while self.pending and self.pending[0][0] <= t:
            self.label_legs = self.pending.pop(0)[1]
        return self.legs, self.label_legs


# ---------------------------------------------------------------- main loop
def log_trial(source, trial, out_dir, duration, auto, seed):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{trial}.csv")
    jams = set()                                  # legs the label says are stuck
    sched = AutoScheduler(seed) if auto else None
    if sched and sched.detached:
        source.set_detached(sched.detached)
    n = 0
    last_hb = last_print = 0.0
    t0 = None
    with open(path, "w", newline="") as f, KeyReader() as keys:
        writer = csv.DictWriter(f, fieldnames=CSV_COLS)
        writer.writeheader()
        while True:
            now = time.monotonic()
            if now - last_hb > HEARTBEAT_S:
                source.heartbeat()
                last_hb = now

            key = keys.get()
            if key == "q":
                break
            if key == "g" and hasattr(source, "toggle_gait"):
                source.toggle_gait()
            if key == "0":
                jams = set()
            elif key in ("1", "2", "3", "4"):
                jams ^= {LEGS[int(key) - 1]}          # press again to mark it free
            if key in ("0", "1", "2", "3", "4") and hasattr(source, "set_jams"):
                source.set_jams(jams)

            msg = source.read()
            if msg is None:
                continue
            t0 = msg["t"] if t0 is None else t0
            t_s = (msg["t"] - t0) / 1000

            if sched:
                physical, jams = sched.update(t_s)
                source.set_jams(physical)

            writer.writerow(telemetry_to_row(msg, trial, jams))
            n += 1

            if not auto and now - last_print > 1.0:
                stuck = "+".join(leg for leg in LEGS if leg in jams) or "nothing"
                print(f"\r t={t_s:6.1f}s  samples={n:6d}  stuck={stuck:12s}", end="", flush=True)
                last_print = now
            if duration and t_s >= duration:
                break
    if hasattr(source, "close"):
        source.close()
    print(f"\nsaved {n} samples -> {path}")
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["fake", "serial", "udp"], default="fake")
    ap.add_argument("--port", default="COM5", help="serial port for --source serial (COM5, /dev/ttyUSB0)")
    ap.add_argument("--robot-ip", default=None, help="--source udp: skip discovery and use this address")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--rate", type=float, default=50, help="fake robot sample rate, Hz")
    ap.add_argument("--out", default="data")
    ap.add_argument("--name", default=None, help="trial name (default: timestamp)")
    ap.add_argument("--duration", type=float, default=0, help="seconds; 0 = until you press q")
    ap.add_argument("--auto", action="store_true", help="fake only: jam legs randomly, label automatically")
    ap.add_argument("--trials", type=int, default=1, help="with --auto: how many trials to generate")
    ap.add_argument("--fast", action="store_true", help="fake only: don't wait for real time")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--no-enc", action="store_true",
                    help="fake only: pretend there are no encoders, like the real robot (ESP32-C3 sketch)")
    args = ap.parse_args()

    if args.auto and args.source != "fake":
        ap.error("--auto only works with --source fake (real labels come from your keypresses)")

    base_seed = args.seed if args.seed is not None else random.randrange(1 << 30)
    for i in range(args.trials):
        seed = base_seed + i
        if args.source == "fake":
            source = FakeSource(seed, args.rate, args.fast, encoders=not args.no_enc)
        elif args.source == "serial":
            source = SerialSource(args.port, args.baud)
        else:
            source = UdpSource(args.robot_ip)
        trial = args.name or time.strftime("trial_%Y%m%d_%H%M%S")
        if args.trials > 1:
            trial = f"{trial}_{i:03d}"
        if not args.auto:
            print("keys: 1-4 = toggle FL/FR/RL/RR stuck, 0 = all free, g = walk on/off, q = stop")
        log_trial(source, trial, args.out, args.duration, args.auto, seed)


if __name__ == "__main__":
    main()
