"""Turn classifier output into actions: wait, try to free the leg, or release it.

Replay a logged trial through the trained model + state machine:
  python decide.py models/stuck_rf.joblib data/some_trial.csv

The same OnlineDetector + ReleaseController run live: feed each telemetry row
to detector.add(row); whenever it returns probabilities, pass them to
controller.update(...) and send whatever commands it returns to the ESP32.
"""
import argparse
from collections import deque

import joblib
import numpy as np
import pandas as pd

from common import LEGS, CLASS_NAMES
from features import add_derived, window_features


def release_threshold(cost_false_release, cost_stuck):
    """Release when p * C_stuck > (1 - p) * C_false  <=>  p > C_false / (C_false + C_stuck)."""
    return cost_false_release / (cost_false_release + cost_stuck)


class OnlineDetector:
    """Keeps the last window of samples; every hop, returns P(class) for classes 0..4."""

    def __init__(self, bundle, rate_hz=50):
        self.model = bundle["model"]
        self.model.n_jobs = 1          # one window at a time: parallelism only adds overhead
        self.features = bundle["features"]
        self.win = int(round(bundle["win_s"] * rate_hz))
        self.hop = int(round(bundle["hop_s"] * rate_hz))
        self.buf = deque(maxlen=self.win)
        self.since = 0

    def add(self, row):
        self.buf.append(row)
        self.since += 1
        if len(self.buf) < self.win or self.since < self.hop:
            return None
        self.since = 0
        w = add_derived(pd.DataFrame(list(self.buf)))
        x = pd.DataFrame([window_features(w)])[self.features]
        p = np.zeros(len(CLASS_NAMES))
        p[self.model.classes_] = self.model.predict_proba(x)[0]   # model may not know every class
        return p


class ReleaseController:
    """NORMAL -> (stuck for N windows) -> UNSTICK -> RECHECK -> freed? NORMAL : retry or RELEASE."""

    def __init__(self, threshold=0.9, n_consec=3, max_unstick=2, unstick_s=1.5, min_legs_after=3):
        self.thr = threshold
        self.n_consec = n_consec
        self.max_unstick = max_unstick
        self.unstick_s = unstick_s
        self.min_legs_after = min_legs_after
        self.attached = [True] * len(LEGS)
        self.state = "NORMAL"
        self.leg = None
        self.count = 0
        self.attempt = 0
        self.t_unstick = 0.0

    def _top_leg(self, p):
        """Most likely stuck leg among legs still attached, and its probability."""
        probs = [p[i + 1] if self.attached[i] else 0.0 for i in range(len(LEGS))]
        i = int(np.argmax(probs))
        return i, probs[i]

    def update(self, t, p):
        """t = seconds, p = probabilities for classes 0..4. Returns a list of commands to send."""
        cmds = []
        if self.state == "NORMAL":
            i, pi = self._top_leg(p)
            if pi >= self.thr and (self.leg is None or i == self.leg):
                self.leg, self.count = i, self.count + 1
            else:
                self.leg, self.count = (i, 1) if pi >= self.thr else (None, 0)
            if self.count >= self.n_consec:
                self.state, self.attempt, self.t_unstick = "UNSTICK", 1, t
                cmds.append({"cmd": "unstick", "leg": LEGS[self.leg], "attempt": 1})

        elif self.state == "UNSTICK":
            # Ignore the classifier while the leg is wiggling: the motion is unusual on purpose.
            if t - self.t_unstick >= self.unstick_s:
                self.state, self.count = "RECHECK", 0

        elif self.state == "RECHECK":
            still = p[self.leg + 1] >= self.thr
            self.count = self.count + 1 if still else self.count - 1
            if self.count <= -self.n_consec:                       # freed
                cmds.append({"cmd": "resume"})
                self.state, self.leg, self.count = "NORMAL", None, 0
            elif self.count >= self.n_consec:                      # still stuck
                if self.attempt < self.max_unstick:
                    self.attempt += 1
                    self.state, self.t_unstick = "UNSTICK", t
                    cmds.append({"cmd": "unstick", "leg": LEGS[self.leg], "attempt": self.attempt})
                elif sum(self.attached) - 1 >= self.min_legs_after:  # safe to drop it
                    cmds.append({"cmd": "detach", "leg": LEGS[self.leg]})
                    self.attached[self.leg] = False
                    self.state, self.leg, self.count = "NORMAL", None, 0
                else:                                              # can't stand without it
                    cmds.append({"cmd": "stop", "reason": f"{LEGS[self.leg]} stuck, too few legs to release"})
                    self.state = "HOLD"                            # wait for a human
        return cmds


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("log", help="one trial CSV to replay")
    ap.add_argument("--cost-false", type=float, default=10, help="cost of dropping a good leg")
    ap.add_argument("--cost-stuck", type=float, default=1, help="cost of staying stuck a bit longer")
    ap.add_argument("--n", type=int, default=3, help="windows in a row before acting")
    args = ap.parse_args()

    bundle = joblib.load(args.model)
    df = pd.read_csv(args.log)
    rate = 1000 / np.median(np.diff(df["t_ms"]))
    thr = release_threshold(args.cost_false, args.cost_stuck)
    print(f"threshold p* = {args.cost_false} / ({args.cost_false} + {args.cost_stuck}) = {thr:.2f}")

    det = OnlineDetector(bundle, rate)
    ctl = ReleaseController(threshold=thr, n_consec=args.n)
    good = bad = 0
    for row in df.to_dict("records"):
        p = det.add(row)
        if p is None:
            continue
        t = (row["t_ms"] - df["t_ms"].iloc[0]) / 1000
        for cmd in ctl.update(t, p):
            truth = CLASS_NAMES[int(row["label"])]
            print(f"t={t:6.2f}s  {cmd}   (true label: {truth})")
            if cmd["cmd"] in ("unstick", "detach"):
                right = truth == f"stuck_{cmd['leg']}"
                good += right
                bad += not right
    print(f"\nactions on the right leg: {good}, on a leg that wasn't stuck: {bad}")


if __name__ == "__main__":
    main()
