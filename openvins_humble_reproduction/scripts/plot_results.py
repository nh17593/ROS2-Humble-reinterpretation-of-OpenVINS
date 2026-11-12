#!/usr/bin/env python3
"""Recreate OpenVINS Figure 3 and Figure 4 metrics from actual simulation logs."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_logs(folder):
    arrays = []
    # State values have a four-component quaternion; std has three orientation errors.
    for name, columns in [("estimate.txt", 19), ("std.txt", 18), ("groundtruth.txt", 19)]:
        data = np.loadtxt(folder/name, comments="#", ndmin=2)
        if data.shape[0] < 2 or data.shape[1] < columns:
            raise ValueError(f"Incomplete {name}: shape {data.shape}")
        if not np.isfinite(data).all():
            raise ValueError(f"Nonfinite values in {name}; inspect simulator log")
        if np.any(np.diff(data[:, 0]) <= 0):
            raise ValueError(f"Non-increasing timestamps in {name}")
        arrays.append(data)
    est, std, gt = arrays
    if len(est) != len(std) or len(est) != len(gt):
        raise ValueError("Log lengths differ; do not silently truncate or interpolate")
    for data in [std, gt]:
        if not np.allclose(est[:, 0], data[:, 0], rtol=0, atol=1e-6):
            raise ValueError("Timestamps differ; simulation logs must share timestamps")
    if np.any(std[:, 1:17] < 0):
        raise ValueError("Standard deviations must be nonnegative")
    for data in [est, gt]:
        if not np.allclose(np.linalg.norm(data[:, 1:5], axis=1), 1, atol=1e-4):
            raise ValueError("Quaternion norms are not approximately one")
    if not np.all(est[:, 18] == gt[:, 18]) or not np.all(est[:, 18] == std[:, 17]):
        raise ValueError("Camera-count columns disagree")
    return est, std, gt


def errors(est, std, gt):
    # OpenVINS JPL rotation matrices are the transpose of SciPy's Hamilton matrices
    # with the same xyzw components. Match ResultSimulation.cpp:
    # e_theta = -Log(R_gt_JPL * R_est_JPL^T).
    relative = Rotation.from_quat(gt[:, 1:5]).inv() * Rotation.from_quat(est[:, 1:5])
    ori = -relative.as_rotvec(degrees=True)
    pos = gt[:, 5:8] - est[:, 5:8]
    offset = est[:, 17] - gt[:, 17]
    return ori, pos, offset


def plot_error(ax, time, error, sigma, ylabel):
    ax.plot(time, error, color="#1565c0", lw=1, label="Error")
    ax.plot(time, 3*sigma, "--", color="#c62828", lw=0.9, label=r"$\pm3\sigma$")
    ax.plot(time, -3*sigma, "--", color="#c62828", lw=0.9)
    ax.axhline(0, color="#888888", lw=0.5)
    ax.set_ylabel(ylabel)
    ax.grid(alpha=0.2)


def analyse(folder):
    est, std, gt = load_logs(folder)
    ori, pos, offset = errors(est, std, gt)
    elapsed = est[:, 0] - est[0, 0]
    meta_file = folder/"metadata.json"
    meta = json.loads(meta_file.read_text()) if meta_file.exists() else {}
    scenario = meta.get("scenario", "unlabelled run")
    seed = meta.get("seed", "unknown")
    label = f"{scenario}; seed {seed}; upstream {meta.get('upstream_commit', 'unknown')[:8]}"
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(3, 2, figsize=(11, 7), sharex=True)
    for index, axis in enumerate("xyz"):
        plot_error(axes[index, 0], elapsed, ori[:, index], np.rad2deg(std[:, 1+index]), f"{axis} error (deg)")
        plot_error(axes[index, 1], elapsed, pos[:, index], std[:, 4+index], f"{axis} error (m)")
    axes[0, 0].set_title("IMU orientation error")
    axes[0, 1].set_title("IMU position error")
    axes[0, 0].legend(loc="best", fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Time since first logged estimate (s)")
    fig.suptitle("Figure 3 reproduction: pose error and uncertainty\n" + label, fontsize=11)
    fig.tight_layout()
    for extension in ["png", "pdf"]:
        fig.savefig(folder/f"figure3_pose_errors.{extension}", dpi=200)
    plt.close(fig)
    # Figure 4 uses only the first ten seconds, as in the authors' evaluator.
    early = elapsed <= 10
    calibrating = bool(np.any(std[:, 16] > 0))
    fig, ax = plt.subplots(figsize=(9, 3.3))
    if calibrating:
        plot_error(ax, elapsed[early], offset[early], std[early, 16], "Time-offset error (s)")
    else:
        ax.plot(elapsed[early], offset[early], color="#1565c0", label="Fixed time-offset error")
        ax.set_ylabel("Time-offset error (s)")
        ax.grid(alpha=0.2)
        ax.text(0.98, 0.92, "Calibration disabled: no estimated uncertainty",
                ha="right", va="top", transform=ax.transAxes, fontsize=8)
    ax.set_xlabel("Time since first logged estimate (s)")
    ax.legend(loc="best", fontsize=8)
    ax.set_title("Figure 4 reproduction: camera-IMU time-offset error\n" + label, fontsize=10)
    fig.tight_layout()
    for extension in ["png", "pdf"]:
        fig.savefig(folder/f"figure4_time_offset.{extension}", dpi=200)
    plt.close(fig)
    result = {
        "samples": len(est), "duration_s": float(elapsed[-1]),
        "scenario": scenario, "seed": seed,
        "position_rmse_m": float(np.sqrt(np.mean(np.sum(pos**2, axis=1)))),
        "orientation_rmse_deg": float(np.sqrt(np.mean(np.sum(ori**2, axis=1)))),
        "position_component_3sigma_coverage_fraction": np.mean(np.abs(pos) <= 3*std[:, 4:7], axis=0).tolist(),
        "orientation_component_3sigma_coverage_fraction": np.mean(np.abs(ori) <= 3*np.rad2deg(std[:, 1:4]), axis=0).tolist(),
        "timeoffset_first10s_rmse_s": float(np.sqrt(np.mean(offset[early]**2))),
        "timeoffset_final_error_s": float(offset[-1]),
        "timeoffset_calibrated": calibrating,
        "timeoffset_first10s_3sigma_coverage_fraction": float(np.mean(np.abs(offset[early]) <= 3*std[early, 16])) if calibrating else None,
        "note": "Unaligned simulated-state RMSE in the common simulator frame. Not the real-data ATE protocol. Marginal 3-sigma coverage is not NEES.",
    }
    (folder/"metrics.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    analyse(args.folder.expanduser().resolve())


if __name__ == "__main__":
    main()
