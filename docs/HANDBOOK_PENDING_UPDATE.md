# Items for the next handoff manual revision

The next manual must record the following project baseline.

- Unified project root on the laptop:
  `/home/yii/Documents/Codex/2026-09-07/new-chat/outputs/G1D_Grasp_Project`
- Expected board deployment root: `/home/unitree/G1D_Grasp_Project`
- Project version: 0.3.3 on the laptop.
- Board startup command: `./g1d-board-start`
- Laptop startup command: `./g1d-laptop-start`
- Both commands are foreground supervisors; internal processes retain separate
  environments and process groups.
- Ctrl+C performs targeted child-process shutdown. No broad `pkill`, hidden
  `nohup`, ARM, trajectory execution, or RH56H1 command is used.
- The canonical table command remains the handoff-manual command:
  `logon_ros2`, `source ~/g1_ws/install/setup.bash`, then
  `python3 ~/下载/add_g1_table_collision.py`. The onsite operator confirmed on
  2026-09-07 that its compensated RViz coordinates remain correct after the
  column raise. `./g1dctl plan` uses the same pinned geometry and remains
  Plan-only.
- Every planning run is written under `runs/<timestamp>`.
- Old plans made before the column raise are quarantined under
  `archive/stale_after_column_raise_15cm`.
- The exact live-board launchers, state reader/package, 15 cm height source,
  RH56H1 controller V4, mapper V5, and UDP driver were exported on 2026-09-07.
  Their source hashes are pinned in `config/onboard_source_provenance.json`.
- Version 0.2.1 removes shell `nounset` while loading ROS/Conda environments;
  Foxy setup files read optional variables and cannot safely run under `set -u`.
- Version 0.2.2 starts AnyGrasp from its SDK root so the proprietary loader can
  resolve `licenseCfg.json` and the `.lic` file as required by the established
  manual startup procedure.
- Version 0.2.3 also promotes the SDK root ahead of the Conda environment's
  `gsnet_versions` entry. Startup logs now print the resolved `gsnet` module
  path; the expected root build is under `grasp_detection/gsnet`, not
  `grasp_detection/gsnet_versions`. The server uses unbuffered output so its
  readiness marker reaches the supervisor immediately.
- Version 0.3.0 collects the laptop startup chain, MoveIt config/URDF sources,
  the actual installed relay source, and the RH56H1 UDP-driver package. Exact
  board-only launch/state/height/controller files still require a live-board
  export; retained snapshots are clearly marked as references.
- Version 0.3.1 imports and verifies that live-board export, promotes the exact
  board scripts into the project, and adds automatic source-hash auditing.
- Version 0.3.2 records the onsite decision that the handoff manual's existing
  compensated table command and `[0.48, 0.00, -0.18]` center remain canonical.
- Version 0.3.3 fixes strict table readback verification: MoveIt normally
  stores the submitted `torso_link` object in `AGV_link` and composes the table
  center into `CollisionObject.pose`. Verification now compares the equivalent
  transformed spatial pose instead of requiring the frame-name string to stay
  unchanged. Live Plan-only passed with 0.000003 m position error and 0.000006
  rad orientation error.
