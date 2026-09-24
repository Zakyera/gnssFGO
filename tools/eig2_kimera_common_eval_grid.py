#!/usr/bin/env python3
"""Resample receiver trajectories onto one Kimera timestamp grid for scoring.

This is evaluation-only SE(3) interpolation, not a receiver factor or state
grid change. Every compared product is evaluated at identical timestamps.
"""

import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def load(path):
    data = np.loadtxt(path)
    if data.ndim != 2 or data.shape[1] != 8:
        raise ValueError(f"bad TUM file: {path}")
    if np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError(f"nonmonotone TUM timestamps: {path}")
    return data


def resample(source, target_times, max_gap=0.25):
    times = source[:, 0]
    rows = []
    for timestamp in target_times:
        right = int(np.searchsorted(times, timestamp))
        if right < len(times) and abs(times[right] - timestamp) < 1e-6:
            rows.append(source[right, 1:].copy())
            continue
        if right == 0 or right == len(times):
            raise ValueError("target outside source support")
        left = right - 1
        span = times[right] - times[left]
        if span <= 0 or span > max_gap:
            raise ValueError(f"interpolation gap {span} exceeds {max_gap}")
        alpha = (timestamp - times[left]) / span
        p = (1.0 - alpha) * source[left, 1:4] + alpha * source[right, 1:4]
        rotations = Rotation.from_quat(source[[left, right], 4:8])
        q = Slerp([0.0, 1.0], rotations)([alpha]).as_quat()[0]
        rows.append(np.r_[p, q])
    return np.asarray(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--grid", type=Path, required=True,
                        help="native Kimera matured TUM with physical timestamps")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--main-start", type=int, default=4)
    parser.add_argument("--main-end", type=int,
                        help="inclusive last source index of one connected component")
    parser.add_argument("--min-samples", type=int, default=100)
    parser.add_argument("products", nargs="+", help="name=trajectory.tum")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    grid = load(args.grid)
    if args.main_start >= len(grid):
        raise ValueError("main component start outside source grid")
    products = {}
    for spec in args.products:
        name, path = spec.split("=", 1)
        if name in products:
            raise ValueError("duplicate product name")
        products[name] = load(Path(path))
    # Common support intersection is selected once, before any scoring.
    minimum = max(values[0, 0] for values in products.values())
    maximum = min(values[-1, 0] for values in products.values())
    last = args.main_end + 1 if args.main_end is not None else len(grid)
    target = grid[args.main_start:last, 0]
    target = target[(target >= minimum - 1e-7) & (target <= maximum + 1e-7)]
    if len(target) < args.min_samples:
        raise ValueError("insufficient common main-chain support")
    for name, values in products.items():
        sampled = resample(values, target)
        with (args.out / f"{name}.tum").open("w") as stream:
            for timestamp, row in zip(target, sampled):
                stream.write(f"{timestamp:.9f} " +
                             " ".join(f"{item:.17g}" for item in row) + "\n")
    print(f"common samples={len(target)} start={target[0]:.9f} end={target[-1]:.9f}")


if __name__ == "__main__":
    main()
