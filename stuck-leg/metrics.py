"""Evaluation numbers that matter for this robot, shared by baseline.py and train.py.

Y and P are n x 4 arrays of 0/1: one row per window, one column per leg (FL FR RL RR),
1 = stuck. att (optional) is the same shape: 1 = that leg is attached.
"""
import numpy as np

from common import LEGS


def stuck_metrics(Y, P, att=None):
    Y, P = np.asarray(Y, dtype=bool), np.asarray(P, dtype=bool)
    if att is not None:                      # never count a detached leg either way
        att = np.asarray(att, dtype=bool)
        Y, P = Y & att, P & att
    normal = ~Y.any(axis=1)
    stuck_pairs = Y.sum()
    return {
        # Robot is fine but we say some leg is stuck -> could drop a good leg. Most important.
        "false_alarm": float(P[normal].any(axis=1).mean()) if normal.any() else float("nan"),
        # A leg is stuck and we say nothing about that leg.
        "missed": float((Y & ~P).sum() / stuck_pairs) if stuck_pairs else float("nan"),
        # Something is stuck, and we also blamed a leg that isn't.
        "wrong_leg": float((P & ~Y)[~normal].any(axis=1).mean()) if (~normal).any() else float("nan"),
        # Exactly the right set of legs (including "none").
        "accuracy": float((Y == P).all(axis=1).mean()),
    }


def report(Y, P, att=None, title=None):
    Y, P = np.asarray(Y, dtype=bool), np.asarray(P, dtype=bool)
    if title:
        print(f"\n=== {title} ===")
    print(f"{'leg':6s}{'precision':>11s}{'recall':>9s}{'stuck windows':>15s}")
    for i, leg in enumerate(LEGS):
        tp, fp, fn = (Y[:, i] & P[:, i]).sum(), (~Y[:, i] & P[:, i]).sum(), (Y[:, i] & ~P[:, i]).sum()
        prec = tp / (tp + fp) if tp + fp else float("nan")
        rec = tp / (tp + fn) if tp + fn else float("nan")
        print(f"{leg:6s}{prec:11.3f}{rec:9.3f}{int(Y[:, i].sum()):15d}")

    # The situations the robot cares about, separately.
    n_stuck = Y.sum(axis=1)
    n_off = (~np.asarray(att, dtype=bool)).sum(axis=1) if att is not None else np.zeros(len(Y), int)
    groups = [("all windows", np.ones(len(Y), bool)), ("one leg stuck", n_stuck == 1),
              ("two legs stuck", n_stuck == 2), ("a leg already off", n_off >= 1)]
    print(f"\n{'situation':20s}{'windows':>9s}{'false alarm':>13s}{'missed':>9s}{'wrong leg':>11s}")
    for name, mask in groups:
        if not mask.any():
            continue
        m = stuck_metrics(Y[mask], P[mask], None if att is None else np.asarray(att)[mask])
        print(f"{name:20s}{int(mask.sum()):9d}{_pct(m['false_alarm']):>13s}{_pct(m['missed']):>9s}{_pct(m['wrong_leg']):>11s}")
    m = stuck_metrics(Y, P, att)
    print(f"\nfalse alarm {m['false_alarm']:.1%} | missed {m['missed']:.1%} | "
          f"wrong leg {m['wrong_leg']:.1%} | exact match {m['accuracy']:.1%}")
    return m


def _pct(x):
    return "-" if x != x else f"{x:.1%}"
