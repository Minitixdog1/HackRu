"""Evaluation numbers that matter for this robot, shared by baseline.py and train.py."""
import numpy as np
from sklearn.metrics import classification_report, confusion_matrix


def stuck_metrics(y, pred):
    y, pred = np.asarray(y), np.asarray(pred)
    normal, stuck = y == 0, y != 0
    return {
        # Robot is fine but we say a leg is stuck -> could drop a good leg. Most important.
        "false_alarm": float((pred[normal] != 0).mean()) if normal.any() else float("nan"),
        # A leg is stuck and we say nothing.
        "missed": float((pred[stuck] == 0).mean()) if stuck.any() else float("nan"),
        # We noticed something is stuck but blamed the wrong leg.
        "wrong_leg": float(((pred[stuck] != 0) & (pred[stuck] != y[stuck])).mean()) if stuck.any() else float("nan"),
        "accuracy": float((pred == y).mean()),
    }


def report(y, pred, names, title=None):
    if title:
        print(f"\n=== {title} ===")
    labels = list(range(len(names)))
    print(classification_report(y, pred, labels=labels, target_names=names, digits=3, zero_division=0))
    cm = confusion_matrix(y, pred, labels=labels)
    width = max(len(n) for n in names) + 2
    print("confusion matrix (rows = true, cols = predicted)")
    print(" " * width + "".join(f"{n:>10}" for n in names))
    for name, row in zip(names, cm):
        print(f"{name:<{width}}" + "".join(f"{v:>10}" for v in row))
    m = stuck_metrics(y, pred)
    print(f"\nfalse alarm {m['false_alarm']:.1%} | missed {m['missed']:.1%} | "
          f"wrong leg {m['wrong_leg']:.1%} | accuracy {m['accuracy']:.1%}")
    return m
