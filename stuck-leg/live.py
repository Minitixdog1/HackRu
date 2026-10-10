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
import csv
import os
import time

import joblib

from common import CLASS_NAMES, CSV_COLS, LEGS, telemetry_to_row
from decide import OnlineDetector, ReleaseController, release_threshold
from logger import KeyReader
from robot_link import RobotLink


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

    thr = release_threshold(args.cost_false, args.cost_stuck)
    det = OnlineDetector(bundle, args.rate)
    ctl = ReleaseController(threshold=thr, n_consec=args.n)
    print(f"release threshold p* = {thr:.2f}; acting after {args.n} windows in a row"
          + ("  [watch only]" if args.watch_only else ""))
    print(__doc__.split("Keys while running:")[1].rstrip())

    writer = log_file = None
    trial = time.strftime("live_%Y%m%d_%H%M%S")
    if args.log:
        os.makedirs(args.log, exist_ok=True)
        path = os.path.join(args.log, f"{trial}.csv")
        log_file = open(path, "w", newline="")
        writer = csv.DictWriter(log_file, fieldnames=CSV_COLS)
        writer.writeheader()
        print(f"logging to {path}")

    if not args.no_auto_walk:
        link.send({"cmd": "gait", "on": 1})
    label = 0
    p_last = None
    n_msgs, rate_t0, rate = 0, time.monotonic(), 0.0
    last_status = 0.0
    walking = False

    try:
        with KeyReader() as keys:
            while True:
                key = keys.get()
                if key == "q":
                    break
                if key == "g":
                    link.send({"cmd": "gait", "on": 0 if walking else 1})
                elif key == " ":
                    link.send({"cmd": "stop"})
                elif key in ("0", "1", "2", "3", "4"):
                    label = int(key)
                    if sim:
                        link.send({"cmd": "sim_jam", "leg": LEGS[label - 1] if label else None})
                elif key == "a":
                    for leg in LEGS:
                        link.send({"cmd": "attach", "leg": leg})
                    ctl = ReleaseController(threshold=thr, n_consec=args.n)
                    print("\nall legs attached, release logic reset")

                msg = link.read()
                if msg is None:
                    if link.seconds_since_data() > 2:
                        print("\nno data from the robot for 2 s; reconnecting...")
                        info = link.connect()
                    continue

                n_msgs += 1
                now = time.monotonic()
                if now - rate_t0 >= 1.0:
                    rate, n_msgs, rate_t0 = n_msgs / (now - rate_t0), 0, now

                if sim:                                    # the sim robot knows the truth
                    label = msg.get("jam", -1) + 1
                row = telemetry_to_row(msg, trial, label)
                if writer:
                    writer.writerow(row)

                was_walking, walking = walking, bool(msg.get("gait", 1))
                if not walking:
                    if was_walking:                        # don't mix standing and walking in a window
                        det = OnlineDetector(bundle, args.rate)
                    p_last = None
                else:
                    if not was_walking and ctl.state == "HOLD":
                        ctl = ReleaseController(threshold=thr, n_consec=args.n)
                    ctl.attached = [bool(a) for a in msg.get("att", [1, 1, 1, 1])]
                    p = det.add(row)
                    if p is not None:
                        p_last = p
                        for cmd in ctl.update(msg["t"] / 1000, p):
                            truth = CLASS_NAMES[label]
                            print(f"\n>> {cmd}   (state {ctl.state}{', truth ' + truth if sim else ''})")
                            if not args.watch_only:
                                link.send(cmd)

                if now - last_status > 0.25:
                    last_status = now
                    att = msg.get("att", [1, 1, 1, 1])
                    legs = " ".join(
                        f"{leg}:{'--off--' if not att[i] else (bar(p_last[i + 1]) if p_last is not None else '........')}"
                        for i, leg in enumerate(LEGS))
                    jam = f" jam={LEGS[msg['jam']]}" if sim and msg.get("jam", -1) >= 0 else ""
                    print(f"\r{'WALK' if walking else 'STOP'} {ctl.state:8s} P(stuck) {legs}  {rate:4.0f} Hz{jam}   ",
                          end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        if log_file:
            log_file.close()
        print("\nmotors stopped, bye")


if __name__ == "__main__":
    main()
