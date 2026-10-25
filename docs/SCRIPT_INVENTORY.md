# Project-specific script inventory

Environment installations, Conda environments, model weights, proprietary
SDK binaries, and ROS build/install directories are external dependencies and
are intentionally not copied here.

## Collected and current on the laptop

- `laptop/startup/start_g1_control.sh`: copied from the active
  `/usr/local/bin/start_g1_control`, then changed only to call the adjacent
  project-owned MoveIt launcher.
- `laptop/startup/start_g1_moveit.sh`: copied from the active
  `/home/yii/.local/bin/start_g1_moveit`.
- `laptop/bridge`: current `g1_d_moveit_bridge` source. The active relay now
  includes time scaling, resampling, speed/increment gates, and final settle.
- `laptop/ros_packages/g1_d_moveit_config`: current MoveIt launch and config
  package source.
- `laptop/ros_packages/g1_d_description`: active URDF sources and package
  metadata. Mesh assets are external non-script assets and are not duplicated.
- `laptop/anygrasp`, `laptop/pipeline`, `laptop/moveit`: current project
  AnyGrasp, planning, diagnostics, and table-scene scripts.
- `hand/laptop/rh56h1_gateway_v2.py`: current laptop UDP gateway.

## Collected and current on the board

- `onboard/trajectory_bridge`: bounded endpoint-feedback V2 bridge source.
- `onboard/vision`: D435 pipeline source.
- `onboard/startup`: exact live-board control/state launchers and trajectory
  bridge environment setup, exported on 2026-09-07.
- `onboard/state`: exact live-board Unitree reader and complete state-bridge ROS
  package source, exported on 2026-09-07.
- `onboard/height/g1d_height_15cm.cpp`: exact time-based 15 cm column source
  used on the board. Collection does not authorize or invoke it.
- `hand/onboard/rh56h1_virtual_gripper_controller_v4.py`: exact live-board
  controller source.
- `hand/onboard/rh56h1_virtual_gripper_mapper_v5.py` and
  `hand/onboard/rh56h1_udp_driver`: verified byte-for-byte against the live
  board export.
- `hand/onboard/service_interfaces`: complete RH56H1 ROS message/service
  definition source copied from the local Inspire SDK tree so future builds do
  not depend on message definitions that exist only in an old install space.

See `docs/START_ENTRY_DEPENDENCIES.md` for the complete four-launcher call
chain and the boundary between retained project source and external runtimes.

The immutable source archive is
`imports/onboard/onboard_script_export_20260907-113353.tar.gz`. Its archive hash
and promoted-file hashes are pinned in `config/onboard_source_provenance.json`.

## Retained references

- `onboard/startup/start_g1_control.reference.sh`: earlier retained launcher.
- `onboard/state_snapshot_20260827`: earlier snapshots; hashes match the
  corresponding 2026-09-07 live state files.
- `onboard/height/g1d_height_control.reference.cpp`: generic SDK target-height
  example, retained only as a reference and not used in operations.

For a future board revision, rerun `tools/collect_onboard_scripts.sh` manually
and compare the new archive before promoting it. The exporter only reads files;
it does not call ROS services or Unitree commands.
