# Board deployment

The board supervisor expects this project at:

```text
/home/unitree/G1D_Grasp_Project
```

Create the board transfer archive on the laptop:

```bash
python3 tools/package_board_bundle.py
```

Transfer the generated archive from `dist` to the board. Before extracting or
replacing any file, keep the robot DISARMED and fully stop the old control and
state processes. Validate the extracted command without starting it:

```bash
cd /home/unitree/G1D_Grasp_Project
./g1d-board-start --check
./g1d-hand-board-start --check
```

The main supervisor uses the installed board workspace and `start_g1_control`.
The separate `g1d-hand-board-start` supervisor launches the RH56H1 driver,
mapper, and controller directly from the project bundle in three isolated Foxy
processes. The board workspace must already provide the `service_interfaces`
message package; the project-pinned RH56H1 driver itself does not need a
separate `colcon build`.

The collected `onboard/trajectory_bridge` directory remains the versioned
source of truth; deploying or rebuilding that ROS package is a separate,
explicit maintenance operation.

The bundle also carries the exact exported board launchers under
`onboard/startup`, the state sources under `onboard/state`, the 15 cm source
under `onboard/height`, and the RH56H1 sources under `hand/onboard`. These are
versioned source copies. Only the separate hand supervisor executes the three
RH56H1 service sources, and starting it does not issue a hand command.

To export exact project-specific scripts that exist only on the live board,
run this separately while the robot remains DISARMED:

```bash
cd /home/unitree/G1D_Grasp_Project
./tools/collect_onboard_scripts.sh
```

This reads source files and writes `runs/onboard_script_export_<timestamp>.tar.gz`.
It does not call a ROS service, publish DDS commands, or stop running services.
