"""A pretend ESP32 on this laptop: same Wi-Fi protocol as the real firmware.

Lets you run the dashboard, commander.py, live.py and logger.py with no
hardware at all. It uses fake_robot.py for the sensor data, so the stuck-leg
model reacts to it exactly as it would to the ESP32's simulated-sensor mode.

  python fake_esp32.py                 # then: python live.py --robot-ip 127.0.0.1
  python dashboard.py --demo           # starts one of these for you

Commands it understands (same as the firmware): hello, hb, gait (on, freq,
turn), stop, unstick, resume, detach, attach, zero, sim_jam.
"""
import argparse
import json
import random
import socket
import threading
import time

from common import LEGS
from fake_robot import FakeRobot

RATE_HZ = 50
HEARTBEAT_TIMEOUT_S = 0.3
PC_IDLE_S = 2.0
UNSTICK_S = 1.5


class FakeEsp32:
    def __init__(self, port=4210, host="0.0.0.0", seed=None, log=print):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.log = log
        self.rng = random.Random(seed)
        self.robot = FakeRobot(seed)
        self.robot.amp = 25.0
        self.robot.freq = 1.5
        self.pc = None
        self.last_pc = 0.0
        self.walking = False
        self.turn = 0.0
        self.attached = [True] * 4
        self.jam = -1
        self.unstick_until = 0.0
        self.last_msg = None
        self.t0 = time.monotonic()
        self._stop = threading.Event()
        self._thread = None

    # ---------------------------------------------------------------- commands
    def _handle(self, msg, addr):
        if self.pc != addr:
            self.log(f"[fake ESP32] PC at {addr[0]}:{addr[1]}")
        self.pc, self.last_pc = addr, time.monotonic()
        cmd = msg.get("cmd", "")
        leg = LEGS.index(msg["leg"]) if msg.get("leg") in LEGS else -1
        if cmd == "hello":
            self._send({"hello": "robotdog", "sim": 1, "ap": 0, "fake": 1})
        elif cmd == "gait":
            self.walking = bool(msg.get("on", 1))
            self.robot.freq = float(msg.get("freq", self.robot.freq))
            self.robot.amp = float(msg.get("amp", self.robot.amp))
            self.turn = max(-1.0, min(1.0, float(msg.get("turn", 0) or 0)))
        elif cmd == "stop":
            self.walking = False
        elif cmd == "unstick" and leg >= 0 and self.walking:
            self.unstick_until = time.monotonic() + UNSTICK_S
        elif cmd == "resume":
            self.unstick_until = 0.0
        elif cmd == "detach" and leg >= 0:
            self.attached[leg] = False
            if self.jam == leg:                 # a dropped leg can't stay stuck
                self.jam = -1
                self.robot.set_stuck(None)
        elif cmd == "attach" and leg >= 0:
            self.attached[leg] = True
        elif cmd == "sim_jam":
            self.jam = leg if leg >= 0 and self.attached[leg] else -1
            self.robot.set_stuck(LEGS[self.jam] if self.jam >= 0 else None)
            if self.jam >= 0 and "s" in msg:
                self.robot.stuck_strength = float(msg["s"])
            elif self.jam >= 0:
                self.robot.stuck_strength = 1.0

    def _send(self, obj):
        if self.pc:
            try:
                self.sock.sendto(json.dumps(obj).encode(), self.pc)
            except OSError:
                pass

    # ---------------------------------------------------------------- loop
    def _tick(self, dt):
        while True:                              # read every waiting command
            try:
                data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, ConnectionResetError):
                break
            try:
                self._handle(json.loads(data), addr)
            except ValueError:
                pass
        now = time.monotonic()
        if self.walking and now - self.last_pc > HEARTBEAT_TIMEOUT_S:
            self.walking = False
            self.log("[fake ESP32] no heartbeat from PC: motors off")

        if self.walking or self.last_msg is None:
            msg = self.robot.step(dt)
            if not self.walking:                 # first message before any gait
                msg["cur"] = [abs(self.rng.gauss(0, 5)) for _ in LEGS]
        else:                                    # standing: legs hold still, motors quiet
            msg = dict(self.last_msg)
            msg["t"] = int(self.robot.t * 1000)
            self.robot.t += dt
            msg["cur"] = [round(abs(self.rng.gauss(0, 5)), 1) for _ in LEGS]
            imu = list(msg["imu"])
            imu[3:6] = [round(self.rng.gauss(0, 1.5), 2) for _ in range(3)]
            msg["imu"] = imu
        if self.walking:
            imu = list(msg["imu"])
            imu[5] = round(imu[5] - 30 * self.turn, 2)   # turning right = negative yaw rate
            msg["imu"] = imu
        for i in range(4):
            if not self.attached[i]:             # detached leg: no motor load, angle frozen
                msg["cur"][i] = round(abs(self.rng.gauss(0, 5)), 1)
                if self.last_msg is not None:
                    msg["enc"][i] = self.last_msg["enc"][i]
                    msg["cmd"][i] = self.last_msg["cmd"][i]
        msg.update(gait=int(self.walking), att=[int(a) for a in self.attached], sim=1, jam=self.jam)
        self.last_msg = msg
        if self.pc and now - self.last_pc < PC_IDLE_S:
            self._send(msg)

    def run(self):
        dt = 1.0 / RATE_HZ
        nxt = time.monotonic()
        while not self._stop.is_set():
            self._tick(dt)
            nxt += dt
            time.sleep(max(0.0, nxt - time.monotonic()))

    def start(self):
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        self.sock.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=4210)
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()
    fake = FakeEsp32(args.port, seed=args.seed)
    print(f"fake ESP32 listening on UDP port {args.port}. Connect with --robot-ip 127.0.0.1 (Ctrl+C to quit)")
    try:
        fake.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
