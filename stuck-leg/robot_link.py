"""Wi-Fi link to the ESP32: find it, keep it alive, read telemetry, send commands.

The ESP32 listens on UDP port 4210. The PC sends {"cmd":"hello"} (broadcast,
plus robotdog.local and 192.168.4.1), the robot answers, and from then on it
streams telemetry back to whichever address the PC talks from. The PC must send
something at least every 300 ms or the robot turns its motors off, so call
read() in a tight loop: it sends the heartbeat for you.

Quick check that the robot is reachable:
  python robot_link.py                  # or: python robot_link.py --robot-ip 192.168.4.1
"""
import argparse
import json
import socket
import time

PORT = 4210
HEARTBEAT_S = 0.1
AP_DEFAULT_IP = "192.168.4.1"   # the ESP32's address when it makes its own network


def _local_broadcasts():
    """Broadcast addresses to try: global, plus x.y.z.255 for each local IPv4 address."""
    addrs = {"255.255.255.255"}
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                addrs.add(ip.rsplit(".", 1)[0] + ".255")
    except OSError:
        pass
    return sorted(addrs)


class RobotLink:
    def __init__(self, robot_ip=None, port=PORT):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("0.0.0.0", 0))           # any free local port; the robot replies to it
        self.sock.settimeout(0.05)
        self.robot = (robot_ip, port) if robot_ip else None
        self.info = {}                           # the robot's hello reply: {"hello", "sim", "ap"}
        self.last_hb = 0.0
        self.last_rx = 0.0

    # ------------------------------------------------------------ sending
    def send(self, obj):
        if self.robot:
            self.sock.sendto(json.dumps(obj).encode(), self.robot)

    def heartbeat(self, force=False):
        now = time.monotonic()
        if force or now - self.last_hb >= HEARTBEAT_S:
            self.send({"cmd": "hb"})
            self.last_hb = now

    # ------------------------------------------------------------ discovery
    def connect(self, wait_s=15):
        """Find the robot. Returns its hello info; raises TimeoutError if nothing answers."""
        targets = [self.robot] if self.robot else self._candidates()
        hello = json.dumps({"cmd": "hello"}).encode()
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            for t in targets:
                try:
                    self.sock.sendto(hello, t)
                except OSError:
                    pass                          # e.g. a broadcast address this machine can't use
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                try:
                    data, addr = self.sock.recvfrom(2048)
                except (socket.timeout, ConnectionResetError):
                    continue
                msg = self._parse(data)
                if msg is None:
                    continue
                self.robot = (addr[0], self.port)
                if "hello" in msg:
                    self.info = msg
                self.last_rx = time.monotonic()
                return self.info or {"hello": "robotdog"}
        raise TimeoutError(
            "no answer from the robot. Check: same Wi-Fi network (or joined 'robotdog'), "
            "firewall allows Python, or pass --robot-ip (shown on the ESP32 serial monitor)")

    def _candidates(self):
        targets = [(b, self.port) for b in _local_broadcasts()] + [(AP_DEFAULT_IP, self.port)]
        try:
            targets.append((socket.gethostbyname("robotdog.local"), self.port))
        except OSError:
            pass
        return targets

    # ------------------------------------------------------------ receiving
    @staticmethod
    def _parse(data):
        try:
            return json.loads(data)
        except (ValueError, UnicodeDecodeError):
            return None

    def read(self):
        """Send the heartbeat if due, then return one telemetry dict, or None if nothing arrived."""
        self.heartbeat()
        try:
            data, addr = self.sock.recvfrom(2048)
        except (socket.timeout, ConnectionResetError):
            return None
        msg = self._parse(data)
        if msg is None:
            return None
        self.last_rx = time.monotonic()
        if "hello" in msg:
            self.info = msg
            return None
        return msg if "imu" in msg else None

    def seconds_since_data(self):
        return time.monotonic() - self.last_rx

    def close(self):
        try:
            self.send({"cmd": "stop"})
        finally:
            self.sock.close()


def main():
    ap = argparse.ArgumentParser(description="Check that the ESP32 is reachable and streaming.")
    ap.add_argument("--robot-ip", default=None)
    args = ap.parse_args()
    link = RobotLink(args.robot_ip)
    info = link.connect()
    print(f"found robot at {link.robot[0]}  {info}")
    n, t0 = 0, time.monotonic()
    try:
        while time.monotonic() - t0 < 3:
            msg = link.read()
            if msg:
                n += 1
                if n == 1:
                    print("first message:", json.dumps(msg)[:200], "...")
        print(f"{n / 3:.0f} messages/second over 3 s")
    finally:
        link.close()


if __name__ == "__main__":
    main()
