"""Rule-based stuck detector: the bar the ML model has to beat.

Leg i is "straining" at a sample when BOTH hold:
    |cmd_i - enc_i| > EPS_DEG     (it isn't getting where it was told)
    cur_i           > CUR_MA      (and the motor is pushing hard)
Leg i is called stuck when it has been straining for at least FRAC of the
last HOLD_MS. (A fraction, not "every sample", because a jammed leg's error
briefly passes through zero each time the command swings past the frozen angle.)
Each leg is judged on its own, so two legs can be stuck at once. Detached legs are never stuck.

Run:  python baseline.py data/
"""
import argparse

import numpy as np

from common import ATT_COLS, LEGS
from features import load_logs, make_windows, sample_rate
from metrics import report

EPS_DEG = 10.0
CUR_MA = 380.0
HOLD_MS = 300
FRAC = 0.7


def baseline_predict(df, eps=EPS_DEG, cur_ma=CUR_MA, hold_ms=HOLD_MS, frac=FRAC):
    """Per-sample stuck flags (n x 4) for every row of df, trial by trial."""
    out = np.zeros((len(df), len(LEGS)), dtype=bool)
    for _, d in df.groupby("trial", sort=False):
        idx = d.index.to_numpy()
        n_hold = max(1, int(round(hold_ms / 1000 * sample_rate(d))))
        for i, leg in enumerate(LEGS):
            strain = (d[f"err_{leg}"].abs() > eps) & (d[f"cur_{leg}"] > cur_ma) & (d[ATT_COLS[i]] > 0)
            out[idx, i] = strain.rolling(n_hold, min_periods=n_hold).mean().fillna(0).to_numpy() >= frac
    return out


def baseline_windows(df, groups, ends, **kw):
    """Baseline's call at the END of each window: what the robot would know at that moment."""
    per_sample = baseline_predict(df, **kw)
    trial_start = {t: d.index[0] for t, d in df.groupby("trial", sort=False)}
    return np.array([per_sample[trial_start[trial] + end] for trial, end in zip(groups, ends)]).reshape(-1, 4)


def window_attached(X):
    return (X[ATT_COLS].to_numpy() > 0.5) if ATT_COLS[0] in X else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data", help="CSV file or folder of CSVs")
    ap.add_argument("--eps", type=float, default=EPS_DEG)
    ap.add_argument("--cur", type=float, default=CUR_MA)
    ap.add_argument("--hold", type=float, default=HOLD_MS)
    args = ap.parse_args()

    df = load_logs(args.data)
    X, Y, groups, ends = make_windows(df)
    pred = baseline_windows(df, groups, ends, eps=args.eps, cur_ma=args.cur, hold_ms=args.hold)
    print(f"Baseline rule: |err| > {args.eps} deg AND current > {args.cur} mA for {FRAC:.0%} of {args.hold:.0f} ms")
    report(Y, pred, window_attached(X))


if __name__ == "__main__":
    main()
