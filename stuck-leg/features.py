"""Load logs and cut them into labeled windows with summary features.

A window is WIN_S seconds of samples; a new one starts every HOP_S seconds.
Each window becomes one row of features and four labels, one per leg
(1 = that leg is stuck). Windows where any leg's label changes partway (less
than PURITY of the samples agree) are dropped, since keypress timing is never exact.
"""
import glob
import os

import numpy as np
import pandas as pd

from common import LEGS, IMU_COLS, TOF_COLS, CUR_COLS, JAM_COLS, ATT_COLS

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
    df = pd.concat([upgrade(pd.read_csv(p)) for p in paths], ignore_index=True)
    return add_derived(df)


def upgrade(df):
    """Older logs have one `label` column and no attached flags: fill them in."""
    if JAM_COLS[0] not in df:
        for i, col in enumerate(JAM_COLS):
            df[col] = (df["label"] == i + 1).astype(int)
    if ATT_COLS[0] not in df:
        for col in ATT_COLS:
            df[col] = 1
    return df


def add_derived(df):
    """Per-leg tracking error, plus each leg compared with the other ATTACHED legs."""
    df = df.copy()
    for col in ATT_COLS:
        if col not in df:
            df[col] = 1
    for leg in LEGS:
        df[f"err_{leg}"] = df[f"cmd_{leg}"] - df[f"enc_{leg}"]
    # "Relative" signals: this leg minus the average of the attached legs. A stuck leg
    # stands out from its siblings even when the whole robot is working harder. A
    # detached leg (no motor load) is left out of the average so it doesn't skew it.
    att = df[ATT_COLS].to_numpy(dtype=float)
    n_att = np.maximum(att.sum(axis=1), 1.0)
    tof_mean = (df[TOF_COLS].to_numpy() * att).sum(axis=1) / n_att
    cur_mean = (df[CUR_COLS].to_numpy() * att).sum(axis=1) / n_att
    for i, leg in enumerate(LEGS):
        df[f"rtof_{leg}"] = (df[f"tof_{leg}"] - tof_mean) * att[:, i]
        df[f"rcur_{leg}"] = (df[f"cur_{leg}"] - cur_mean) * att[:, i]
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
    for col in ATT_COLS:                    # which legs are on: a detached leg looks "stuck-ish"
        feats[col] = w[col].to_numpy().mean() if col in w else 1.0
    return feats


def sample_rate(df):
    dt = np.median(np.diff(df["t_ms"].to_numpy())) / 1000
    return 1.0 / dt if dt > 0 else 50.0


def make_windows(df, win_s=WIN_S, hop_s=HOP_S, purity=PURITY, keep_impure=False):
    """Return X (features), Y (n x 4 stuck flags), groups (trial id), end index of each window."""
    rows, labels, groups, ends = [], [], [], []
    for trial, d in df.groupby("trial", sort=False):
        d = d.reset_index(drop=True)
        rate = sample_rate(d)
        win = max(2, int(round(win_s * rate)))
        hop = max(1, int(round(hop_s * rate)))
        jam = d[JAM_COLS].to_numpy()
        for start in range(0, len(d) - win + 1, hop):
            frac = jam[start:start + win].mean(axis=0)          # share of samples saying "stuck", per leg
            agree = np.maximum(frac, 1 - frac)
            if agree.min() < purity and not keep_impure:
                continue
            w = d.iloc[start:start + win]
            rows.append(window_features(w))
            labels.append((frac >= 0.5).astype(int))
            groups.append(trial)
            ends.append(w.index[-1])
    X = pd.DataFrame(rows)
    return X, np.array(labels).reshape(-1, 4), np.array(groups), np.array(ends)
