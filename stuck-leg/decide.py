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

from common import ATT_COLS, JAM_COLS, LEGS, CLASS_NAMES, detach_allowed, detach_refusal
from features import add_derived, upgrade, window_features


def release_threshold(cost_false_release, cost_stuck):
    """Release when p * C_stuck > (1 - p) * C_false  <=>  p > C_false / (C_false + C_stuck)."""
    return cost_false_release / (cost_false_release + cost_stuck)


class OnlineDetector:
    """Keeps the last window of samples; every hop, returns 5 numbers:
    [P(no leg stuck), P(FL stuck), P(FR stuck), P(RL stuck), P(RR stuck)].

    Per-leg models (train.py now) give each leg its own probability, so two legs can
    both be near 1. Older one-leg-at-a-time models are still supported.
    """

    def __init__(self, bundle, rate_hz=50):
        self.multilabel = bool(bundle.get("multilabel"))
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
        if self.multilabel:
            for i, proba in enumerate(self.model.predict_proba(x)):
                classes = list(self.model.classes_[i])
                p[i + 1] = proba[0, classes.index(1)] if 1 in classes else 0.0
            p[0] = float(np.prod(1 - p[1:]))
        else:
            p[self.model.classes_] = self.model.predict_proba(x)[0]   # model may not know every class
        return p


class ReleaseController:
    """NORMAL -> (stuck for N windows) -> UNSTICK -> RECHECK -> freed? NORMAL : retry or RELEASE.

    Hysteresis: a leg has to reach `threshold` to be acted on, but once the robot is
    working on it, it only counts as freed when its probability falls below
    `free_below`. So a leg hovering at 80% isn't mistaken for "freed".

    Per-leg model (per_leg=True): a leg is stuck when its own probability >= threshold.
    Two stuck legs are handled one at a time, most confident first.
    Older one-label model (per_leg=False): "something is stuck" = the attached legs'
    probabilities add up to >= threshold, and "which leg" = the most likely one if it
    holds at least min_share of the probability.

    Detaching follows common.detach_allowed: one leg, or a diagonal pair (FL+RR, FR+RL).
    Anything else stops the robot and waits for a human (state HOLD).
    """

    def __init__(self, threshold=0.75, n_consec=3, max_unstick=2, unstick_s=1.5, min_share=0.3, per_leg=True,
                 free_below=0.5):
        self.per_leg = per_leg
        self.thr = threshold
        self.free_below = min(free_below, threshold)
        self.n_consec = n_consec
        self.max_unstick = max_unstick
        self.unstick_s = unstick_s
        self.min_share = min_share
        self.attached = [True] * len(LEGS)
        self.state = "NORMAL"
        self.leg = None
        self.count = 0
        self.attempt = 0
        self.t_unstick = 0.0

    def _top_leg(self, p):
        """Most likely stuck attached leg, its probability, and P(any attached leg is stuck)."""
        probs = [p[i + 1] if self.attached[i] else 0.0 for i in range(len(LEGS))]
        i = int(np.argmax(probs))
        return i, probs[i], float(sum(probs))

    def _stuck(self, p, leg, level=None):
        """Is `leg` stuck in this window, at probability `level` (default: the threshold)?"""
        level = self.thr if level is None else level
        if not self.attached[leg]:
            return False
        if self.per_leg:
            return p[leg + 1] >= level
        _, _, p_any = self._top_leg(p)
        return p_any >= level and p[leg + 1] >= self.min_share

    def update(self, t, p):
        """t = seconds, p = probabilities for classes 0..4. Returns a list of commands to send."""
        cmds = []
        if self.state == "NORMAL":
            i, _, _ = self._top_leg(p)
            confident = self._stuck(p, i)
            if confident and (self.leg is None or i == self.leg):
                self.leg, self.count = i, self.count + 1
            else:
                self.leg, self.count = (i, 1) if confident else (None, 0)
            if self.count >= self.n_consec:
                self.state, self.attempt, self.t_unstick = "UNSTICK", 1, t
                cmds.append({"cmd": "unstick", "leg": LEGS[self.leg], "attempt": 1})

        elif self.state == "UNSTICK":
            # Ignore the classifier while the leg is wiggling: the motion is unusual on purpose.
            if t - self.t_unstick >= self.unstick_s:
                self.state, self.count = "RECHECK", 0

        elif self.state == "RECHECK":
            still = self._stuck(p, self.leg, self.free_below)
            self.count = self.count + 1 if still else self.count - 1
            if self.count <= -self.n_consec:                       # freed
                cmds.append({"cmd": "resume"})
                self.state, self.leg, self.count = "NORMAL", None, 0
            elif self.count >= self.n_consec:                      # still stuck
                if self.attempt < self.max_unstick:
                    self.attempt += 1
                    self.state, self.t_unstick = "UNSTICK", t
                    cmds.append({"cmd": "unstick", "leg": LEGS[self.leg], "attempt": self.attempt})
                elif detach_allowed(self.attached, self.leg):      # safe to drop it
                    cmds.append({"cmd": "detach", "leg": LEGS[self.leg]})
                    self.attached[self.leg] = False
                    self.state, self.leg, self.count = "NORMAL", None, 0
                else:                                              # dropping it would leave it unable to balance
                    cmds.append({"cmd": "stop", "reason": detach_refusal(self.attached, self.leg)})
                    self.state = "HOLD"                            # wait for a human
        return cmds


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("log", help="one trial CSV to replay")
    ap.add_argument("--cost-false", type=float, default=3, help="cost of dropping a good leg")
    ap.add_argument("--cost-stuck", type=float, default=1, help="cost of staying stuck a bit longer")
    ap.add_argument("--n", type=int, default=3, help="windows in a row before acting")
    args = ap.parse_args()

    bundle = joblib.load(args.model)
    df = upgrade(pd.read_csv(args.log))
    rate = 1000 / np.median(np.diff(df["t_ms"]))
    thr = release_threshold(args.cost_false, args.cost_stuck)
    print(f"threshold p* = {args.cost_false} / ({args.cost_false} + {args.cost_stuck}) = {thr:.2f}")

    det = OnlineDetector(bundle, rate)
    ctl = ReleaseController(threshold=thr, n_consec=args.n, per_leg=det.multilabel)
    good = bad = 0
    for row in df.to_dict("records"):
        p = det.add(row)
        if p is None:
            continue
        ctl.attached = [bool(row[c]) for c in ATT_COLS]
        t = (row["t_ms"] - df["t_ms"].iloc[0]) / 1000
        for cmd in ctl.update(t, p):
            stuck = [leg for leg, c in zip(LEGS, JAM_COLS) if row[c]]
            print(f"t={t:6.2f}s  {cmd}   (truly stuck: {'+'.join(stuck) or 'nothing'})")
            if cmd["cmd"] in ("unstick", "detach"):
                right = cmd["leg"] in stuck
                good += right
                bad += not right
    print(f"\nactions on the right leg: {good}, on a leg that wasn't stuck: {bad}")


if __name__ == "__main__":
    main()
