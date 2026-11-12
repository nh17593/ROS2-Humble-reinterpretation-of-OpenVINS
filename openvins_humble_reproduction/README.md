# OpenVINS reproduction in ROS 2 Humble

Selected paper: Geneva, P., Eckenhoff, K., Lee, W., Yang, Y. and Huang, G.
(2020), "OpenVINS: A Research Platform for Visual-Inertial Estimation", IEEE
International Conference on Robotics and Automation (ICRA).

Authors' code: https://github.com/rpng/open_vins

Documentation: https://docs.openvins.com/

## Status and deliverables

This kit prepares a reproducibility experiment using the authors' estimator.
Its Python analysis and configuration generation have been checked locally.
The ROS 2 build and estimator runs have NOT been executed here: the preparation
environment has no ROS 2, colcon, CMake, or Docker. There are no reproduced
experimental results in this archive. Generate them on your Humble machine
with the commands below before writing the final results and reflection.

The upstream repository already includes ROS 2 integration and a Humble Docker
configuration. Describe your contribution as configuring, integrating, and
evaluating the authors' implementation under Humble. This kit adds experiment
orchestration and analysis; it does not implement a new MSCKF estimator or
claim to port the original 2020 code from ROS 1.

## What the method does

The estimator propagates an on-manifold EKF using IMU acceleration and angular
velocity. Image-feature observations constrain a sliding window of cloned IMU
poses through the Multi-State Constraint Kalman Filter (MSCKF). Selected
long-lived features may remain in the state. First-Estimates Jacobians (FEJ)
help preserve the correct unobservable directions. Camera intrinsics,
camera-to-IMU extrinsics, and camera-IMU time offset can be estimated online.

For the paper's simulation experiment, a smooth SE(3) B-spline defines the true
trajectory. The simulator generates noisy IMU and feature-coordinate
measurements, allowing the estimator's errors and uncertainty to be compared
directly with known truth. It feeds feature coordinates directly and initializes
the inertial state from truth. It does not test image feature detection/tracking
or autonomous initialization. These are properties of this upstream simulation
path, and must be disclosed in the report.

## Two figures to reproduce

Both are on PDF page 5 of the supplied seven-page Geneva2020ICRA.pdf.

| Paper figure | Generated files | Measurement and explanation |
| --- | --- | --- |
| Figure 3 | figure3_pose_errors.png and .pdf | IMU orientation error (degrees) and position error (meters), with positive and negative three-sigma marginal uncertainty bounds. Look for drift, overconfidence, and differences with calibration enabled. |
| Figure 4 | figure4_time_offset.png and .pdf | Estimated minus true camera-IMU time-offset error (seconds), with three-sigma bounds, over the first ten seconds. Look for correction of the deliberately perturbed time offset and explain residual bias or lack of convergence. |

These are two distinct figures in the paper. Do not replace them with an RViz
screenshot or count separate components of one figure as separate experiments.
The scripts recreate the metrics and visual structure from newly generated
logs; they do not trace or digitize the authors' published curves.

## Experiment choices

| Setting | Value |
| --- | --- |
| ROS / OS target | ROS 2 Humble / Ubuntu 22.04 |
| Upstream revision | 69488123ed9362dd44b6f28e7f4680abbff1442b |
| Camera count and rate | One camera, 10 Hz |
| IMU rate | 400 Hz |
| Sliding-window size | 11 clones |
| Feature count | 100 simulated tracks per frame |
| Maximum in-state landmarks | 50 |
| Visual measurement noise | One pixel standard deviation |
| FEJ | Enabled |
| Online camera calibration | Enabled or disabled according to scenario |
| Online IMU intrinsics and gravity sensitivity | Disabled; these are later extensions |
| Trajectory | udel_gore.txt from the pinned authors' repository |

The 2020 simulation rates, window, track count, and landmark count come from
Section V-A, PDF page 4. The supplied paper does not establish the exact
trajectory filename or historical random seed for Figures 3-4. This kit chooses
the documented udel_gore trajectory explicitly. The pinned modern code, current
calibration/noise files, perturbation priors, update scheduling, and solver
dependencies may differ from the original experiments. Therefore describe the
output as a reproduction of the experimental methodology, with exact numerical
replication unverified. Do not expect Table I values to match automatically.

## Install and build

Start on Ubuntu 22.04 with ROS 2 Humble already installed. Extract this archive
and open a terminal in the `openvins_humble_reproduction` folder.

```bash
bash scripts/setup_humble.sh "$HOME/openvins_humble_ws"
source /opt/ros/humble/setup.bash
source "$HOME/openvins_humble_ws/install/setup.bash"
ros2 pkg executables ov_msckf
```

The last command should list `run_simulation`. The setup script installs
dependencies and builds the original estimator at the pinned revision. It uses
a dedicated workspace and refuses to change an existing modified checkout.
It does not edit your shell startup files. You do not need Gazebo, a TurtleBot,
a physical camera, or a downloaded image bag for these simulation figures.

## Run the first experiment

```bash
python3 scripts/run_experiments.py \
  --workspace "$HOME/openvins_humble_ws" \
  --scenario bad_calib_on --runs 1 --output results_first
```

This starts the actual ROS 2 `ov_msckf/run_simulation` executable with perturbed
camera calibration, enables online camera calibration, and writes complete
estimate, standard-deviation, and ground-truth logs. It then creates both
figures automatically. No GUI is required. Do not pass the estimator config as
`--params-file`: it uses OpenCV-style YAML, not ROS 2 parameter-file syntax.

Inspect:

```text
results_first/bad_calib_on/seed_001/
  console.log
  estimator_config.yaml
  metadata.json
  estimate.txt
  std.txt
  groundtruth.txt
  figure3_pose_errors.png
  figure3_pose_errors.pdf
  figure4_time_offset.png
  figure4_time_offset.pdf
  metrics.json
```

`metadata.json` records source revision, exact command, random seed, trajectory
and calibration hashes, generated settings, and process status. Preserve the
logs as experimental evidence. Existing result folders are never overwritten;
use a new `--output` name when rerunning.

## Compare the four calibration cases

```bash
python3 scripts/run_experiments.py \
  --workspace "$HOME/openvins_humble_ws" \
  --scenario all --runs 20 --output results_20runs
```

This requests 80 actual runs, matching Table I's twenty-run count for each of
four cases. It does not reproduce the paper's Table I ATE/NEES evaluator.

| Folder | Initial camera calibration | Online camera calibration |
| --- | --- | --- |
| true_calib_on | True | Enabled |
| true_calib_off | True | Disabled |
| bad_calib_on | Perturbed | Enabled |
| bad_calib_off | Perturbed | Disabled |

The same seed is reused across cases for a paired comparison. The bad/off case
may diverge or terminate; record failures as failures and discuss their rate.
Never replace missing outcomes with zeros. The runner keeps logs and returns
a nonzero exit status if a process, timeout, or analysis fails. The timeout is
1800 wall-clock seconds per run; increase `--timeout` if your machine needs it.

For a first check, use `--scenario all --runs 1` with a new output directory.
For representative figures, use the preselected bad/on seed 1 and state that
choice; do not pick only the most favorable seed after looking at results.

## Replot and interpret

```bash
python3 scripts/plot_results.py results_first/bad_calib_on/seed_001
```

The analysis follows the error signs in the authors' `ResultSimulation.cpp`:
position is truth minus estimate; time offset is estimate minus truth;
orientation is `-Log(R_truth_JPL * R_estimate_JPL^T)`, converted to degrees.
Quaternions use xyzw order and the OpenVINS JPL convention. Standard-deviation
columns store standard deviations, not variances, and have one fewer component
than the quaternion state. The parser validates timestamps, shapes, quaternion
norms, camera counts, finite values, and nonnegative standard deviations.

`metrics.json` reports unaligned simulated position/orientation RMSE, marginal
three-sigma coverage by component, and time-offset error. The estimator and
truth already share a simulator frame. These summaries are not automatically
the paper's real-dataset ATE protocol. Marginal standard deviations do not
contain the full covariance needed to reproduce NEES; this kit does not
approximate full NEES from diagonal terms. A single trace within three-sigma
bounds is suggestive, not proof of consistency; inspect multiple runs.

If the generated plots disagree with the paper, discuss the actual direction
and magnitude of the differences. Do not claim that online calibration always
improves every run; observability, excitation, seeds, and numerical differences
all matter. If the simulator outputs nonfinite values, the analysis fails
instead of plotting misleading numbers.

## Debugging and demonstration

- `ros2` or package missing: source both Humble and your workspace in the same
  terminal; ensure `ros2 pkg prefix ov_msckf` points inside this workspace.
- Missing or empty log files: inspect `console.log`. The ROS 2 visualizer reads
  save flags and output paths from ROS parameters; the runner passes these
  explicitly in addition to its estimator YAML.
- Build runs out of memory: use one compiler job as well, for example prefix
  the build command with `CMAKE_BUILD_PARALLEL_LEVEL=1`.
- Import errors: use Ubuntu's `/usr/bin/python3` and the apt-installed numerical
  libraries, outside a conflicting Conda environment.
- No convergence: verify bad/on really enables all three camera calibration
  flags, inspect motion excitation, and compare seeds without editing truth.

A short demonstration can show `printenv ROS_DISTRO`, the executable list,
the running experiment and logs, both generated figures, and `metrics.json`.
An RViz trajectory view is optional; numerical error curves are the evidence
for Figures 3-4. Capture the real build and run output on your machine.

## Report and appendix

Use `report_template.txt` for the required one-page reflection AFTER completing
the runs. Replace every bracketed field with real observations. Put the two
generated figures in the appendix with captions naming the paper figure,
scenario, seed, settings, units, uncertainty bounds, and limitations. Include
the exact commands, source revision, logs, and failure counts in further
appendices. Keep the main report to one page in your chosen document format.

## References and implementation provenance

- Paper: https://pgeneva.com/downloads/papers/Geneva2020ICRA.pdf
- Authors' publication listing: https://udel.edu/~pgeneva/publications/
- Humble container: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/Dockerfile_ros2_22_04
- Simulator: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/run_simulation.cpp
- Log format: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/ros/ROSVisualizerHelper.cpp
- Error conventions: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_eval/src/calc/ResultSimulation.cpp

OpenVINS remains the authors' GPL-3.0 code. This archive downloads it at setup
time and does not redistribute the upstream estimator or claim authorship of
it. Preserve its existing license and notices if submitting a source checkout.
