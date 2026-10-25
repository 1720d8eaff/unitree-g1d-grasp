# G1 D AnyGrasp MoveIt RH56H1 Unified Project

This directory consolidates the current G1-D left-arm grasp stack without
changing the separation required by ROS 2 Foxy, ROS 2 Humble, Conda, and
Unitree DDS.  Source files were copied from the working handoff locations; the
original files remain untouched.

## Current status

- The onboard bounded endpoint-feedback V2 source is present and pinned by
  SHA-256 in `config/endpoint_feedback_v2.json`.
- The 245-point taught motion and its metadata are under `data/taught`.
- The verified AnyGrasp orientation reference is under `data/references`.
- The exact 2026-09-07 live-board startup, state, 15 cm height, and RH56H1
  sources are collected and hash-pinned in
  `config/onboard_source_provenance.json`.
- Plans created before the 15 cm column raise are quarantined under
  `archive/stale_after_column_raise_15cm` and must never be executed.
- The active raised-column geometry retains the handoff manual's compensated
  `torso_link` table coordinates. The onsite operator confirmed on 2026-09-07
  that this RViz table command is correct and must remain canonical.
- Table verification accepts MoveIt's normal conversion of the submitted
  `torso_link` object into its `AGV_link` planning frame, but still compares the
  transformed position, orientation, primitive type, and dimensions.
- `g1dctl` version 1 does not expose real execution, ARM/DISARM, hand CLOSE,
  or visual adjustment.
- The consolidated Plan-only path keeps the live AnyGrasp position but uses
  the accepted 2026-09-09 manual-teach TCP orientation and joint endpoint as
  the second-stage IK reference. Based on the manually recorded wrist-yaw
  motion (`-7.92°..+24.97°`), the approach uses the rounded `-10°..+30°`
  envelope. MoveIt first solves the final target under the operator-requested
  `-5°..+5°` endpoint constraint, then uses those solutions to seed the
  overhead solve.
- Second-stage reversal is ranked against the preferred `0.40 rad` value and
  only rejected at the gross `1.20 rad` boundary. Collision, robot joint-limit,
  Cartesian-completeness, and full RViz-review checks remain mandatory.

## Two main startup commands

After this project has been deployed to `/home/unitree/G1D_Grasp_Project` on
the board, start all board-side services in one foreground terminal:

```bash
cd /home/unitree/G1D_Grasp_Project
./g1d-board-start
```

Start all laptop-side services in one foreground terminal:

```bash
cd /home/yii/Documents/Codex/2026-09-07/new-chat/outputs/G1D_Grasp_Project
./g1d-laptop-start
```

Each command keeps its internal processes isolated, prefixes their live output,
writes separate logs, prevents a duplicate supervisor, and stops only its own
process groups when Ctrl+C is pressed. These two main commands remain separate
from the RH56H1 services and do not perform ARM or send a trajectory.

Start the hand services separately, using one additional terminal per machine:

```bash
# board: one terminal supervises driver + mapper + controller
cd /home/unitree/G1D_Grasp_Project
./g1d-hand-board-start

# laptop: one terminal runs the UDP-to-RS485 gateway
cd /home/yii/Documents/Codex/2026-09-07/new-chat/outputs/G1D_Grasp_Project
./g1d-hand-laptop-start
```

Starting either hand supervisor does not send PREOPEN/CLOSE/RELEASE.

## First commands

Run the offline project audit:

```bash
./g1dctl doctor
```

Show the required board and laptop terminals:

```bash
./g1dctl terminals
```

If the physical setup or compensation changes in the future, activate a newly
measured table-top Z explicitly:

```bash
./g1dctl geometry-set \
  --table-top-z-m MEASURED_VALUE \
  --note "measurement method and date" \
  --confirm-measured
```

Then, from a laptop ROS 2 Humble terminal with MoveIt, AnyGrasp, and the
selector already running:

```bash
./g1dctl plan
```

This command creates a timestamped run directory, adds and reads back the
verified table collision object, and runs only the unified Plan-only pipeline.

To reproduce the last known planning policy for RViz comparison only:

```bash
./g1dctl plan --previous-preview
```

This explicit compatibility mode remains Plan-only. Strict live orientation,
exact roll, multi-IK search, and the combined-reversal gate remain the default
when `--previous-preview` is omitted.

## Explicit reviewed grasp cycle

After a fresh `./g1dctl plan`, the saved-path executor can add the RH56H1 and a
post-grasp lift without changing the reviewed main trajectory.  It first plans
a collision-checked Cartesian lift of 5 cm along `torso_link` +Z, publishes the
main path and lift together to RViz, and waits for an exact per-run confirmation
phrase.  Only after confirmation does it send PREOPEN, execute the main path,
start CLOSE one second before the main path's nominal endpoint, verify arrival,
and execute the lift. The one-second timer begins from the board's observed
`executing` status, not from the earlier MoveIt request. The hand remains
closed afterwards.

```bash
G1D_RUN_DIR=/absolute/path/to/the/new/run \
python3 laptop/pipeline/execute_full_taught_to_anygrasp_continuous.py \
  --execute-saved \
  --grasp-cycle \
  --object-width-m 0.05 \
  --accept-reviewed-target-drift \
  --accept-known-combined-reversal
```

Set `--object-width-m` to the measured object width within the calibrated
`0.025`–`0.078 m` range; `0.05` is only the example for a 50 mm object.
The reviewed-target override is bounded to 15 mm translation and 20 degrees
of approach-axis change. It is available only for this interactive RViz-reviewed
grasp-cycle mode; larger changes still require a fresh plan.

This mode never sends ARM, DISARM, or RELEASE. Keep the robot DISARMED while
the combined RViz preview is being generated and reviewed; ARM externally only
when the script is waiting for its exact confirmation phrase. Any failed scene,
collision, start-state, moving-target, endpoint, hand-command, or lift check
stops the remaining sequence.

## Read-only full-motion manual calibration

The original taught trajectory and its recorder remain unchanged. Record a
complete manually guided motion from the hanging-arm state to the accepted TCP
grasp pose with:

```bash
./g1dctl teach-calibrate
```

Press Enter to start, slowly guide the arm to the correctly aligned TCP pose,
hold the final pose for at least one second, and press Enter again. The report
finds the recorded sample nearest the existing taught endpoint and treats it as
the first/second-segment seam. It reports both full-motion and second-segment
metrics, but marks only the extracted second-segment values for planner tuning.
The command is read-only and never changes planner gates automatically.

The accepted run `teach-20260909-152516` has now been reviewed and promoted to
`data/references/left_grasp_manual_teach_calibration_20260909.json`. Only the
second-stage planner consumes it; the original recorder and 245-point taught
trajectory remain unchanged. Every plan takes position from one fresh stable
AnyGrasp window, adds the measured `torso_link` TCP position bias, and uses the
accepted taught palm orientation. It never replaces the live position with the
taught absolute `torso_link` position, and it does not rotate the calibration
with AnyGrasp orientation jitter. The taught endpoint joint vector remains an
IK seed, and the frozen live AnyGrasp sample remains the reference for
pre-execution movement checks.

## Directory map

- `config`: safety, endpoint feedback, external paths, and geometry profiles.
- `data`: immutable raw teaching data, current taught trajectory, and verified
  orientation reference.
- `laptop/pipeline`: current planning, checking, diagnostics, and teaching
  modules, including the previously external 2026-09-01 base dependency.
- `laptop/anygrasp`: AnyGrasp server and candidate-A selector.
- `laptop/moveit`: configurable work-table PlanningScene node.
- `laptop/bridge`: laptop trajectory relay ROS package.
- `laptop/startup`: collected laptop control and MoveIt launchers used by the
  laptop supervisor.
- `laptop/ros_packages`: collected project-specific MoveIt configuration and
  active robot-description sources; installed ROS environments stay external.
- `onboard/trajectory_bridge`: board-side split frontend/backend package with
  bounded endpoint-feedback V2.
- `onboard/vision`: D435 detection and localization pipeline.
- `onboard/startup`, `onboard/state`, `onboard/height`: exact live-board
  launcher, state, and 15 cm height sources. Earlier snapshots and generic
  references remain clearly named as such.
- `hand`: collected RH56H1 gateway, mapper, and complete UDP-driver sources.
- `runs`: one directory per new Plan-only run.
- `archive`: historical or geometry-invalidated artifacts.
- `docs`: handoff manual and retained bridge reference material.

## Maintenance rules

1. Do not edit files in dated Codex output folders or `~/下载`; make changes in
   this project and record them in version control.
2. Do not overwrite saved plans.  Use one timestamped directory per run.
3. Treat geometry profile changes as invalidating all earlier plans.
4. Keep ARM/DISARM outside application code and under onsite operator control.
5. Preserve board ROS and Unitree DDS process isolation.
6. Do not enable RH56H1 or visual adjustment until endpoint V2 acceptance and
   physical TCP confirmation both pass.
7. Use `docs/SCRIPT_INVENTORY.md` as the script-completeness ledger. Do not
   claim a retained snapshot is the current board source until hashes are
   compared against a live-board export.
