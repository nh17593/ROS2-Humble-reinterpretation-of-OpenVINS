#!/usr/bin/env python3
"""Run the real OpenVINS simulator through ROS 2; never generate fake results."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import signal
import shutil
import subprocess
import sys
import time

PIN = "69488123ed9362dd44b6f28e7f4680abbff1442b"
SCENARIOS = {
    "bad_calib_on": (True, True),
    "true_calib_on": (False, True),
    "true_calib_off": (False, False),
    "bad_calib_off": (True, False),
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def configure(repo, folder, scenario, seed, trajectory):
    base = repo / "config/rpng_sim"
    text = (base / "estimator_config.yaml").read_text()
    perturbed, calibrate = SCENARIOS[scenario]
    settings = {
        "use_fej": True, "use_stereo": False, "max_cameras": 1,
        "max_clones": 11, "max_slam": 50, "num_pts": 100,
        "max_slam_in_update": 50, "max_msckf_in_update": 1000,
        "calib_cam_extrinsics": calibrate,
        "calib_cam_intrinsics": calibrate,
        "calib_cam_timeoffset": calibrate,
        # These IMU calibration extensions postdate the 2020 paper.
        "calib_imu_intrinsics": False, "calib_imu_g_sensitivity": False,
        "sim_freq_cam": 10, "sim_freq_imu": 400,
        "sim_seed_state_init": seed, "sim_seed_preturb": seed,
        "sim_seed_measurements": seed, "sim_do_perturbation": perturbed,
        "sim_traj_path": str(trajectory), "num_opencv_threads": 0,
        "track_frequency": 10.0, "try_zupt": False,
        "up_msckf_sigma_px": 1.0, "up_slam_sigma_px": 1.0,
        "save_total_state": True,
        "relative_config_imu": str(base / "kalibr_imu_chain.yaml"),
        "relative_config_imucam": str(base / "kalibr_imucam_chain.yaml"),
    }
    for key, value in settings.items():
        text, count = re.subn(r"^" + re.escape(key) + r":.*$",
                              lambda _: f"{key}: {json.dumps(value)}", text,
                              flags=re.MULTILINE)
        if count != 1:
            raise ValueError(f"Expected exactly one config entry: {key}")
    config = folder / "estimator_config.yaml"
    config.write_text(text)
    return config, settings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path.home()/"openvins_humble_ws")
    parser.add_argument("--output", type=Path, default=Path("results"))
    parser.add_argument("--scenario", choices=[*SCENARIOS, "all"], default="bad_calib_on")
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--first-seed", type=int, default=1)
    parser.add_argument("--trajectory", type=Path, help="Alternative upstream trajectory text file")
    parser.add_argument("--timeout", type=float, default=1800, help="Per-run wall-clock seconds")
    args = parser.parse_args()
    if args.runs < 1 or args.first_seed < 0 or args.timeout <= 0:
        parser.error("Runs and timeout must be positive; seed must be nonnegative")
    if os.environ.get("ROS_DISTRO") != "humble" or not shutil.which("ros2"):
        parser.error("Source /opt/ros/humble/setup.bash and your workspace install/setup.bash first")
    repo = (args.workspace.expanduser().resolve() / "src/open_vins")
    try:
        head = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    except subprocess.CalledProcessError:
        parser.error(f"No OpenVINS checkout in {repo}")
    if head != PIN:
        parser.error(f"Expected pinned revision {PIN}; found {head}")
    dirty = subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain"], text=True)
    if dirty.strip():
        parser.error("Upstream checkout has changes; use a clean checkout for this experiment")
    trajectory = (args.trajectory or repo/"ov_data/sim/udel_gore.txt").expanduser().resolve()
    if not trajectory.is_file():
        parser.error(f"Missing trajectory: {trajectory}")
    # Refuse to accidentally execute a binary installed by another workspace.
    prefix = subprocess.check_output(["ros2", "pkg", "prefix", "ov_msckf"], text=True).strip()
    if not Path(prefix).resolve().is_relative_to(args.workspace.expanduser().resolve()/"install"):
        parser.error(f"ov_msckf resolves to another workspace: {prefix}")
    scenarios = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    failures = 0
    for scenario in scenarios:
        for seed in range(args.first_seed, args.first_seed + args.runs):
            folder = args.output.expanduser().resolve()/scenario/f"seed_{seed:03d}"
            folder.mkdir(parents=True, exist_ok=False)
            config, settings = configure(repo, folder, scenario, seed, trajectory)
            command = ["ros2", "run", "ov_msckf", "run_simulation", str(config),
                       "--ros-args", "-p", "save_total_state:=true"]
            for key, name in [("filepath_est", "estimate.txt"),
                              ("filepath_std", "std.txt"),
                              ("filepath_gt", "groundtruth.txt")]:
                # ROS2Visualizer reads these from ROS parameters, not the YAML parser.
                command.extend(["-p", f"{key}:={folder/name}"])
            metadata = {
                "scenario": scenario, "seed": seed, "upstream_commit": head,
                "ros_distro": os.environ["ROS_DISTRO"], "platform": platform.platform(),
                "python": sys.version, "command": command,
                "trajectory": str(trajectory), "trajectory_sha256": sha256(trajectory),
                "config_sha256": sha256(config), "overrides": settings,
                "calibration_sha256": {name: sha256(repo/"config/rpng_sim"/name)
                    for name in ["kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml"]},
                "scope": "Modern upstream implementation of 2020 simulation methodology; exact historical match unverified",
            }
            (folder/"metadata.json").write_text(json.dumps(metadata, indent=2)+"\n")
            print(f"Running {scenario}, seed {seed}; log: {folder/'console.log'}", flush=True)
            start = time.monotonic()
            try:
                with (folder/"console.log").open("w") as logfile:
                    proc = subprocess.Popen(command, cwd=args.workspace.expanduser().resolve(),
                                            stdout=logfile, stderr=subprocess.STDOUT,
                                            start_new_session=True)
                    try:
                        proc.wait(timeout=args.timeout)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGTERM)
                        try:
                            proc.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(proc.pid, signal.SIGKILL)
                            proc.wait()
                        raise
                code = proc.returncode
                status = "completed" if code == 0 else "process_failed"
            except subprocess.TimeoutExpired:
                code, status = None, "timeout"
            metadata.update(returncode=code, status=status, elapsed_s=time.monotonic()-start)
            (folder/"metadata.json").write_text(json.dumps(metadata, indent=2)+"\n")
            if status == "completed":
                try:
                    subprocess.run([sys.executable, str(Path(__file__).with_name("plot_results.py")),
                                    str(folder)], check=True)
                except subprocess.CalledProcessError:
                    metadata["status"] = "analysis_failed"
                    (folder/"metadata.json").write_text(json.dumps(metadata, indent=2)+"\n")
                    failures += 1
            else:
                failures += 1
                print(f"Run ended with {status}; inspect console.log", file=sys.stderr)
    print(f"Finished with {failures} failed or unanalysed runs. Raw logs are preserved.")
    return int(failures > 0)


if __name__ == "__main__":
    raise SystemExit(main())
