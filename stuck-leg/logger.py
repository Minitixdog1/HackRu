"""Log robot telemetry to CSV and label it live with keypresses.

Keys while logging:
  1-4  leg FL / FR / RL / RR is stuck now
  0    nothing is stuck
  q    stop and save

Each run = one trial = one CSV file in data/. Keep trials short (1-3 min) and
vary them: speed, surface, battery, how the leg gets stuck.

Sources:
  --source fake     built-in fake robot (pressing 1-4 really "jams" that leg)
  --source serial   ESP32 over USB, newline-delimited JSON (--port COM5 or /dev/ttyUSB0)
  --source udp      ESP32 over Wi-Fi, one JSON message per UDP packet (--port 4210)

Generate a fake dataset with no keyboard at all:
  python logger.py --source fake --auto --trials 40 --duration 60 --fast
"""
import argparse
import csv
import json
import os
import random
import socket
import sys
import time

from common import CSV_COLS, LEGS, CLASS_NAMES, telemetry_to_row

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
    def __init__(self, seed, rate_hz, fast):
        from fake_robot import FakeRobot
        self.robot = FakeRobot(seed)
        self.dt = 1.0 / rate_hz
        self.fast = fast
        self._next = time.monotonic()

    def read(self):
        if not self.fast:                       # pace at real time
            self._next += self.dt
            time.sleep(max(0.0, self._next - time.monotonic()))
        return self.robot.step(self.dt)

    def set_stuck(self, leg):                   # only the fake robot can be "jammed" by software
        self.robot.set_stuck(leg)

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
    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.robot_addr = None                  # learned from the first packet

    def read(self):
        try:
            data, addr = self.sock.recvfrom(2048)
        except socket.timeout:
            return None
        self.robot_addr = addr
        try:
            return json.loads(data)
        except json.JSONDecodeError:
            return None

    def heartbeat(self):
        if self.robot_addr:
            self.sock.sendto(b'{"cmd":"hb"}', self.robot_addr)


# ---------------------------------------------------------------- auto labels (fake only)
class AutoScheduler:
    """Randomly jams legs for 1.5-4 s, then frees them for 3-8 s.

    The label lags the real change by 0.1-0.35 s, like a person's keypress would,
    so the fake labels are as imperfect as real ones.
    """

    def __init__(self, seed):
        self.rng = random.Random(seed)
        self.leg = None
        self.until = self.rng.uniform(3, 8)
        self.label_leg = None
        self.pending = []                 # (time the label catches up, leg)

    def update(self, t):
        """Return (leg physically stuck now, leg the label says is stuck)."""
        if t >= self.until:
            if self.leg is None:
                self.leg = self.rng.choice(LEGS)
                # Most jams are short; about 1 in 8 is "really stuck" (8-12 s).
                long_jam = self.rng.random() < 0.125
                self.until = t + (self.rng.uniform(8, 12) if long_jam else self.rng.uniform(1.5, 4.0))
            else:
                self.leg = None
                self.until = t + self.rng.uniform(3.0, 8.0)
            self.pending.append((t + self.rng.uniform(0.1, 0.35), self.leg))
        while self.pending and self.pending[0][0] <= t:
            self.label_leg = self.pending.pop(0)[1]
        return self.leg, self.label_leg


# ---------------------------------------------------------------- main loop
def log_trial(source, trial, out_dir, duration, auto, seed):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{trial}.csv")
    label = 0
    sched = AutoScheduler(seed) if auto else None
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
            if key in ("0", "1", "2", "3", "4"):
                label = int(key)
                if hasattr(source, "set_stuck"):
                    source.set_stuck(LEGS[label - 1] if label else None)

            msg = source.read()
            if msg is None:
                continue
            t0 = msg["t"] if t0 is None else t0
            t_s = (msg["t"] - t0) / 1000

            if sched:
                leg, label_leg = sched.update(t_s)
                source.set_stuck(leg)
                label = LEGS.index(label_leg) + 1 if label_leg else 0

            writer.writerow(telemetry_to_row(msg, trial, label))
            n += 1

            if not auto and now - last_print > 1.0:
                print(f"\r t={t_s:6.1f}s  samples={n:6d}  label={CLASS_NAMES[label]:9s}", end="", flush=True)
                last_print = now
            if duration and t_s >= duration:
                break
    print(f"\nsaved {n} samples -> {path}")
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["fake", "serial", "udp"], default="fake")
    ap.add_argument("--port", default="4210", help="serial port (COM5, /dev/ttyUSB0) or UDP port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--rate", type=float, default=50, help="fake robot sample rate, Hz")
    ap.add_argument("--out", default="data")
    ap.add_argument("--name", default=None, help="trial name (default: timestamp)")
    ap.add_argument("--duration", type=float, default=0, help="seconds; 0 = until you press q")
    ap.add_argument("--auto", action="store_true", help="fake only: jam legs randomly, label automatically")
    ap.add_argument("--trials", type=int, default=1, help="with --auto: how many trials to generate")
    ap.add_argument("--fast", action="store_true", help="fake only: don't wait for real time")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()

    if args.auto and args.source != "fake":
        ap.error("--auto only works with --source fake (real labels come from your keypresses)")

    base_seed = args.seed if args.seed is not None else random.randrange(1 << 30)
    for i in range(args.trials):
        seed = base_seed + i
        if args.source == "fake":
            source = FakeSource(seed, args.rate, args.fast)
        elif args.source == "serial":
            source = SerialSource(args.port, args.baud)
        else:
            source = UdpSource(int(args.port))
        trial = args.name or time.strftime("trial_%Y%m%d_%H%M%S")
        if args.trials > 1:
            trial = f"{trial}_{i:03d}"
        if not args.auto:
            print("keys: 1-4 = FL/FR/RL/RR stuck, 0 = free, q = stop")
        log_trial(source, trial, args.out, args.duration, args.auto, seed)


if __name__ == "__main__":
    main()
