# Start entry dependency closure

This ledger records every project-specific file reached by the four root
startup commands. All listed source files are retained in this repository.
Paths under `/opt/ros`, ROS `install` spaces, Conda environments, CycloneDDS,
the Unitree SDK, the AnyGrasp SDK, and model checkpoints are runtime/toolchain
dependencies rather than project source and are listed separately below.

## Root entry points

| Entry point | Supervisor configuration | Direct project processes |
| --- | --- | --- |
| `g1d-board-start` | `config/processes_board.json` | `onboard/startup/start_g1_control.sh`, `onboard/vision/detect_and_locate_d435_pipeline_v4_noalign.py` |
| `g1d-laptop-start` | `config/processes_laptop.json` | `laptop/startup/start_g1_control.sh`, `laptop/anygrasp/server_slanted_stable.py`, `laptop/anygrasp/anygrasp_left_auto_selector.py` |
| `g1d-hand-board-start` | `config/processes_hand_board.json` | `onboard/startup/run_rh56h1_service.sh` and the three sources under `hand/onboard` |
| `g1d-hand-laptop-start` | `config/processes_hand_laptop.json` | `hand/laptop/rh56h1_gateway_v2.py` |

All four entry points call the retained `tools/process_supervisor.py`.

## Nested project source retained here

- Board control: `onboard/startup/start_g1_control.sh`,
  `onboard/startup/start_g1_state.sh`,
  `onboard/startup/g1_trajectory_bridge_env.sh`, `onboard/state`, and
  `onboard/trajectory_bridge`.
- Laptop control: `laptop/startup/start_g1_moveit.sh`, `laptop/bridge`, and the
  project-specific packages under `laptop/ros_packages`.
- RH56H1: the gateway under `hand/laptop`; driver, mapper, controller, and the
  complete ROS message-definition package `hand/onboard/service_interfaces`
  under `hand/onboard`.
- Perception and planning: the full `laptop/anygrasp`, `laptop/moveit`, and
  `laptop/pipeline` trees plus `onboard/vision`.

The board main launcher deliberately resolves the already installed
`start_g1_control` command, and its nested launcher resolves the installed
`start_g1_state` entry point. Their exact collected source copies and all
project-specific nested sources are retained at the paths above so a future
developer is not left with only the installed entry points.

## External runtime dependencies not vendored

- ROS 2 Foxy on the board and ROS 2 Humble on the laptop.
- The built `/home/unitree/ros2_ws/install` and `/home/yii/g1_ws/install`
  spaces. Their project-specific package sources are retained here; generated
  build/install artifacts are not portable source.
- CycloneDDS installations and machine-specific XML network configuration.
- The board Unitree SDK and the `tv` / `tv_jetson` Conda environments.
- The licensed AnyGrasp SDK and its model checkpoint. The project-owned server
  and selection code is retained, but the SDK and weights must be installed
  separately under the paths recorded in `config/external_paths.json`.
- PySerial and the physical `/dev/ttyUSB0` device used by the laptop gateway.

The copied `service_interfaces` package came from
`/home/yii/Robot/yinshi/SDK/src/ros2/src/service_interfaces`. Its upstream
package metadata currently declares the license as `TODO`; preserve that
provenance and resolve licensing before redistribution outside this project.
