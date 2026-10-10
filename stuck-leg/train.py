"""Train the stuck-leg classifier and compare it with the rule-based baseline.

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

from baseline import baseline_windows
from common import CLASS_NAMES
from features import HOP_S, WIN_S, load_logs, make_windows
from metrics import report, stuck_metrics


def make_model(args):
    return RandomForestClassifier(
        n_estimators=args.trees,
        max_depth=args.max_depth,
        min_samples_leaf=2,
        class_weight="balanced",   # stuck windows are rarer than normal ones
        n_jobs=-1,
        random_state=args.seed,
    )


def cross_validate(X, y, groups, args):
    n_trials = len(set(groups))
    folds = min(args.cv, n_trials)
    rows = []
    for k, (tr, te) in enumerate(GroupKFold(n_splits=folds).split(X, y, groups)):
        model = make_model(args).fit(X.iloc[tr], y[tr])
        m = stuck_metrics(y[te], model.predict(X.iloc[te]))
        rows.append(m)
        print(f"fold {k + 1}: false alarm {m['false_alarm']:.1%}  missed {m['missed']:.1%}  "
              f"wrong leg {m['wrong_leg']:.1%}  accuracy {m['accuracy']:.1%}")
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
    X, y, groups, ends = make_windows(df)
    n_trials = len(set(groups))
    print(f"{len(X)} windows from {n_trials} trials, {X.shape[1]} features")
    print("windows per class: " + ", ".join(f"{n}={(y == i).sum()}" for i, n in enumerate(CLASS_NAMES)))

    if args.cv:
        cross_validate(X, y, groups, args)
        return

    if args.random_split:
        print("\n!! random split by WINDOW: overlapping neighbours land on both sides; scores will be too high")
        tr, te = train_test_split(np.arange(len(X)), test_size=args.test_frac, random_state=args.seed, stratify=y)
    else:
        if n_trials < 4:
            raise SystemExit("need at least 4 trials to hold some out; log more trials")
        splitter = GroupShuffleSplit(n_splits=1, test_size=args.test_frac, random_state=args.seed)
        tr, te = next(splitter.split(X, y, groups))
        print(f"train trials: {len(set(groups[tr]))}, test trials: {len(set(groups[te]))}")

    model = make_model(args).fit(X.iloc[tr], y[tr])
    ml = report(y[te], model.predict(X.iloc[te]), CLASS_NAMES, "Random forest (held-out test)")

    base_pred = baseline_windows(df, groups[te], ends[te])
    base = report(y[te], base_pred, CLASS_NAMES, "Rule-based baseline (same test windows)")

    print("\n=== Side by side (lower is better except accuracy) ===")
    for k in ("false_alarm", "missed", "wrong_leg", "accuracy"):
        print(f"  {k:12s} model {ml[k]:7.1%}   baseline {base[k]:7.1%}")

    imp = pd.Series(model.feature_importances_, index=X.columns).sort_values(ascending=False)
    print("\nTop 15 features (what the model leans on)")
    for name, v in imp.head(15).items():
        print(f"  {name:22s} {v:.3f}")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    joblib.dump({"model": model, "features": list(X.columns), "win_s": WIN_S,
                 "hop_s": HOP_S, "classes": CLASS_NAMES}, args.out)
    print(f"\nsaved model -> {args.out}")

    if args.export_c:
        try:
            import m2cgen
        except ImportError:
            raise SystemExit("pip install m2cgen to export C code")
        c_path = os.path.splitext(args.out)[0] + ".c"
        with open(c_path, "w") as f:
            f.write(m2cgen.export_to_c(model))
        print(f"saved C code -> {c_path} ({os.path.getsize(c_path) / 1024:.0f} KB; keep it small for the ESP32)")


if __name__ == "__main__":
    main()
