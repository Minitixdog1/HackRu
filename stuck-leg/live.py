"""Run the stuck-leg model live against the ESP32 over Wi-Fi.

  python live.py                          # find the robot, walk, detect, unstick/detach on its own
  python live.py --robot-ip 192.168.4.1   # skip discovery
  python live.py --watch-only             # show predictions but never send unstick/detach
  python live.py --log data/              # also save every sample as a labeled training CSV

Keys while running:
  g      start / stop walking
  space  stop motors now
  1-4    sim robot: jam leg FL/FR/RL/RR   (real robot: label that leg as stuck in the log)
  0      sim robot: free the leg          (real robot: label "nothing stuck")
  a      reattach all legs and reset the release logic
  q      quit (motors stop)
"""
import argparse
import os
import time

import joblib

from common import LEGS
from logger import KeyReader
from robot_link import RobotLink
from runner import LiveRunner


def bar(p, width=8):
    n = int(round(p * width))
    return "#" * n + "." * (width - n)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="models/stuck_rf.joblib")
    ap.add_argument("--robot-ip", default=None)
    ap.add_argument("--rate", type=float, default=50, help="telemetry rate set in the firmware, Hz")
    ap.add_argument("--cost-false", type=float, default=10, help="cost of dropping a good leg")
    ap.add_argument("--cost-stuck", type=float, default=1, help="cost of staying stuck a bit longer")
    ap.add_argument("--n", type=int, default=3, help="confident windows in a row before acting")
    ap.add_argument("--watch-only", action="store_true", help="never send unstick / detach")
    ap.add_argument("--no-auto-walk", action="store_true", help="don't start walking on connect")
    ap.add_argument("--log", default=None, help="folder to save a labeled CSV of this run")
    args = ap.parse_args()

    if not os.path.exists(args.model):
        raise SystemExit(f"no model at {args.model}. Train one first:  python train.py data/")
    bundle = joblib.load(args.model)

    link = RobotLink(args.robot_ip)
    print("looking for the robot...")
    info = link.connect()
    sim = bool(info.get("sim"))
    print(f"connected to {link.robot[0]} ({'SIMULATED sensors' if sim else 'real sensors'})")

    runner = LiveRunner(link, bundle, rate=args.rate, cost_false=args.cost_false, cost_stuck=args.cost_stuck,
                        n_consec=args.n, watch_only=args.watch_only, log_dir=args.log,
                        on_event=lambda text: print(f"\n>> {text}"))
    print(f"release threshold p* = {runner.thr:.2f}; acting after {args.n} windows in a row"
          + ("  [watch only]" if args.watch_only else ""))
    if args.log:
        print(f"logging to {runner.log_path}")
    print(__doc__.split("Keys while running:")[1].rstrip())

    if not args.no_auto_walk:
        runner.send({"cmd": "gait", "on": 1})
    last_status = 0.0
    try:
        with KeyReader() as keys:
            while True:
                key = keys.get()
                if key == "q":
                    break
                if key == "g":
                    runner.send({"cmd": "gait", "on": 0 if runner.walking else 1})
                elif key == " ":
                    runner.send({"cmd": "stop"})
                elif key in ("0", "1", "2", "3", "4"):
                    runner.label = int(key)
                    if sim:
                        runner.send({"cmd": "sim_jam", "leg": LEGS[runner.label - 1] if runner.label else None})
                elif key == "a":
                    for leg in LEGS:
                        runner.send({"cmd": "attach", "leg": leg})
                    runner.reset_controller()
                    print("\nall legs attached, release logic reset")

                msg = runner.step()
                now = time.monotonic()
                if msg is None or now - last_status < 0.25:
                    continue
                last_status = now
                att = msg.get("att", [1, 1, 1, 1])
                p = runner.p_last
                legs = " ".join(
                    f"{leg}:{'--off--' if not att[i] else (bar(p[i + 1]) if p is not None else '........')}"
                    for i, leg in enumerate(LEGS))
                jam = f" jam={LEGS[msg['jam']]}" if sim and msg.get("jam", -1) >= 0 else ""
                print(f"\r{'WALK' if runner.walking else 'STOP'} {runner.ctl.state:8s} P(stuck) {legs}"
                      f"  {runner.hz:4.0f} Hz{jam}   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        runner.close()
        print("\nmotors stopped, bye")


if __name__ == "__main__":
    main()
