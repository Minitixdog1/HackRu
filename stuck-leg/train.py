"""Train the stuck-leg model and compare it with the rule-based baseline.

The model answers "is this leg stuck?" separately for each of the 4 legs, so it
can flag two legs at once and knows what a detached leg looks like.

  python train.py data/                  # train, evaluate on held-out TRIALS, save model
  python train.py data/ --cv 5           # 5-fold cross-validation by trial (better with few trials)
  python train.py data/ --random-split   # the WRONG way, to see how leakage inflates the score
  python train.py data/ --trees 20 --max-depth 8 --export-c   # small model -> C code for the ESP32
"""
import argparse
import os

import joblib
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import GroupKFold, GroupShuffleSplit, train_test_split

from baseline import baseline_windows, window_attached
from common import CMD_COLS, ENC_COLS, LEGS
from features import HOP_S, WIN_S, load_logs, make_windows
from metrics import report, stuck_metrics

DECISION = 0.5      # P(stuck) above this counts as "stuck" in the evaluation tables


def make_model(args):
    return RandomForestClassifier(
        n_estimators=args.trees,
        max_depth=args.max_depth,
        min_samples_leaf=2,
        class_weight="balanced",   # stuck windows are rarer than normal ones (balanced per leg)
        n_jobs=-1,
        random_state=args.seed,
    )


def leg_probabilities(model, X):
    """n x 4 array: P(leg is stuck) for each window and leg."""
    out = np.zeros((len(X), len(LEGS)))
    for i, proba in enumerate(model.predict_proba(X)):
        classes = list(model.classes_[i])
        if 1 in classes:
            out[:, i] = proba[:, classes.index(1)]
    return out


def cross_validate(X, Y, groups, args):
    folds = min(args.cv, len(set(groups)))
    rows = []
    att = window_attached(X)
    for k, (tr, te) in enumerate(GroupKFold(n_splits=folds).split(X, Y, groups)):
        model = make_model(args).fit(X.iloc[tr], Y[tr])
        m = stuck_metrics(Y[te], leg_probabilities(model, X.iloc[te]) >= DECISION, None if att is None else att[te])
        rows.append(m)
        print(f"fold {k + 1}: false alarm {m['false_alarm']:.1%}  missed {m['missed']:.1%}  "
              f"wrong leg {m['wrong_leg']:.1%}  exact {m['accuracy']:.1%}")
    summary = pd.DataFrame(rows)
    print("\nmean +/- std over folds")
    for col in summary.columns:
        print(f"  {col:12s} {summary[col].mean():.1%} +/- {summary[col].std():.1%}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data", help="folder of CSV logs (or one CSV)")
    ap.add_argument("--test-frac", type=float, default=0.25, help="fraction of TRIALS held out")
    ap.add_argument("--trees", type=int, default=300)
    ap.add_argument("--max-depth", type=int, default=None)
    ap.add_argument("--cv", type=int, default=0, help="k-fold cross-validation by trial instead of one split")
    ap.add_argument("--random-split", action="store_true", help="split by window (leaky, for comparison only)")
    ap.add_argument("--out", default="models/stuck_rf.joblib")
    ap.add_argument("--export-c", action="store_true", help="also write the model as C code (needs m2cgen)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    df = load_logs(args.data)
    X, Y, groups, ends = make_windows(df)
    n_trials = len(set(groups))
    att = window_attached(X)
    print(f"{len(X)} windows from {n_trials} trials, {X.shape[1]} features")
    # No encoders (ESP32-C3 robot, logger --no-enc): the logged angle always equals the command.
    encoders = not np.allclose(df[ENC_COLS].to_numpy(), df[CMD_COLS].to_numpy(), atol=0.01)
    print("encoder data: " + ("yes" if encoders else "none (angle = command), learning from current, IMU and ToF"))
    print("stuck windows per leg: " + ", ".join(f"{leg}={int(Y[:, i].sum())}" for i, leg in enumerate(LEGS))
          + f";  two legs at once: {int((Y.sum(axis=1) == 2).sum())};  with a leg off: {int((~att).any(axis=1).sum())}")

    if args.cv:
        cross_validate(X, Y, groups, args)
        return

    if args.random_split:
        print("\n!! random split by WINDOW: overlapping neighbours land on both sides; scores will be too high")
        tr, te = train_test_split(np.arange(len(X)), test_size=args.test_frac, random_state=args.seed)
    else:
        if n_trials < 4:
            raise SystemExit("need at least 4 trials to hold some out; log more trials")
        splitter = GroupShuffleSplit(n_splits=1, test_size=args.test_frac, random_state=args.seed)
        tr, te = next(splitter.split(X, Y, groups))
        print(f"train trials: {len(set(groups[tr]))}, test trials: {len(set(groups[te]))}")

    model = make_model(args).fit(X.iloc[tr], Y[tr])
    P = leg_probabilities(model, X.iloc[te]) >= DECISION
    ml = report(Y[te], P, att[te], "Random forest (held-out test)")

    base_pred = baseline_windows(df, groups[te], ends[te])
    base = report(Y[te], base_pred, att[te], "Rule-based baseline (same test windows)")

    print("\n=== Side by side (lower is better except exact match) ===")
    for k, name in (("false_alarm", "false alarm"), ("missed", "missed"), ("wrong_leg", "wrong leg"),
                    ("accuracy", "exact match")):
        print(f"  {name:12s} model {ml[k]:7.1%}   baseline {base[k]:7.1%}")

    imp = pd.Series(model.feature_importances_, index=X.columns).sort_values(ascending=False)
    print("\nTop 15 features (what the model leans on)")
    for name, v in imp.head(15).items():
        print(f"  {name:22s} {v:.3f}")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    joblib.dump({"model": model, "features": list(X.columns), "win_s": WIN_S, "hop_s": HOP_S,
                 "legs": LEGS, "multilabel": True, "encoders": encoders}, args.out)
    print(f"\nsaved model -> {args.out}")

    if args.export_c:
        try:
            import m2cgen
            c_path = os.path.splitext(args.out)[0] + ".c"
            with open(c_path, "w") as f:
                f.write(m2cgen.export_to_c(model))
            print(f"saved C code -> {c_path} ({os.path.getsize(c_path) / 1024:.0f} KB; keep it small for the ESP32)")
        except ImportError:
            print("pip install m2cgen to export C code")
        except Exception as e:
            print(f"couldn't export C code for a per-leg model: {e}")


if __name__ == "__main__":
    main()
