"""Load logs and cut them into labeled windows with summary features.

A window is WIN_S seconds of samples; a new one starts every HOP_S seconds.
Each window becomes one row of features and one label. Windows whose samples
are mostly one label (>= PURITY) are kept; windows straddling a label change
are dropped, since your keypress timing is never exact.
"""
import glob
import os

import numpy as np
import pandas as pd

from common import LEGS, IMU_COLS, TOF_COLS, CUR_COLS, ENC_COLS, CMD_COLS

WIN_S = 0.5
HOP_S = 0.25
PURITY = 0.8


def load_logs(paths_or_dir):
    """Load one CSV, a list of CSVs, or every CSV in a folder into one DataFrame."""
    if isinstance(paths_or_dir, str) and os.path.isdir(paths_or_dir):
        paths = sorted(glob.glob(os.path.join(paths_or_dir, "*.csv")))
    elif isinstance(paths_or_dir, str):
        paths = [paths_or_dir]
    else:
        paths = list(paths_or_dir)
    if not paths:
        raise SystemExit(f"no CSV logs found in {paths_or_dir}")
    df = pd.concat([pd.read_csv(p) for p in paths], ignore_index=True)
    return add_derived(df)


def add_derived(df):
    """Per-leg tracking error e = cmd - enc, plus each leg relative to the others."""
    df = df.copy()
    for leg in LEGS:
        df[f"err_{leg}"] = df[f"cmd_{leg}"] - df[f"enc_{leg}"]
    # "Relative" signals: this leg minus the average of all legs. A stuck leg
    # stands out from its siblings even when the whole robot is working harder.
    tof_mean = df[TOF_COLS].mean(axis=1)
    cur_mean = df[CUR_COLS].mean(axis=1)
    for leg in LEGS:
        df[f"rtof_{leg}"] = df[f"tof_{leg}"] - tof_mean
        df[f"rcur_{leg}"] = df[f"cur_{leg}"] - cur_mean
    return df


ERR_COLS = [f"err_{leg}" for leg in LEGS]
REL_COLS = [f"rtof_{leg}" for leg in LEGS] + [f"rcur_{leg}" for leg in LEGS]
SIGNALS = IMU_COLS + TOF_COLS + CUR_COLS + ERR_COLS + REL_COLS


def window_features(w):
    """Summary features for one window (a DataFrame slice)."""
    feats = {}
    for col in SIGNALS:
        x = w[col].to_numpy()
        feats[f"{col}_mean"] = x.mean()
        feats[f"{col}_std"] = x.std()
        feats[f"{col}_min"] = x.min()
        feats[f"{col}_max"] = x.max()
        feats[f"{col}_delta"] = x[-1] - x[0]
    for col in ERR_COLS:                    # size of the error matters, not its sign
        feats[f"{col}_absmean"] = np.abs(w[col].to_numpy()).mean()
    return feats


def sample_rate(df):
    dt = np.median(np.diff(df["t_ms"].to_numpy())) / 1000
    return 1.0 / dt if dt > 0 else 50.0


def make_windows(df, win_s=WIN_S, hop_s=HOP_S, purity=PURITY, keep_impure=False):
    """Return X (features), y (label), groups (trial id), end index of each window."""
    rows, labels, groups, ends = [], [], [], []
    for trial, d in df.groupby("trial", sort=False):
        d = d.reset_index(drop=True)
        rate = sample_rate(d)
        win = max(2, int(round(win_s * rate)))
        hop = max(1, int(round(hop_s * rate)))
        for start in range(0, len(d) - win + 1, hop):
            w = d.iloc[start:start + win]
            counts = w["label"].value_counts()
            label, frac = counts.index[0], counts.iloc[0] / len(w)
            if frac < purity and not keep_impure:
                continue
            rows.append(window_features(w))
            labels.append(int(label))
            groups.append(trial)
            ends.append(w.index[-1])
    X = pd.DataFrame(rows)
    return X, np.array(labels), np.array(groups), np.array(ends)
