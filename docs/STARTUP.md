# Main startup and separate RH56H1 startup

Always start the board command first. The laptop launcher verifies board-side
trajectory topic endpoints and will refuse to become ready when the board chain
is absent.

## Board command

Run on the board:

```bash
cd /home/unitree/G1D_Grasp_Project
./g1d-board-start
```

It starts two isolated children:

1. the existing `start_g1_control` chain in the full Foxy and matching Cyclone
   environment, with bounded endpoint feedback V2 explicitly enabled;
2. the D435 pipeline in the `tv_jetson` Conda environment.

The command does not start the RH56H1 chain. It does not ARM or send a
trajectory. If either required child exits, the supervisor stops the other
child and returns an error.

## Laptop command

Run on the laptop:

```bash
cd /home/yii/Documents/Codex/2026-09-07/new-chat/outputs/G1D_Grasp_Project
./g1d-laptop-start
```

It starts three isolated children:

1. the collected `laptop/startup/start_g1_control.sh` launcher for relay,
   MoveIt, and RViz; it calls the adjacent collected `start_g1_moveit.sh`;
2. the AnyGrasp server in the `anygrasp` Conda environment;
3. the candidate-A selector in the Humble and Cyclone environment.

The command does not start the RH56H1 gateway, plan, or execute. Use Ctrl+C in
the supervisor terminal to stop only the processes created by that supervisor.

## Separate RH56H1 commands

Start the complete board-side hand chain in one additional board terminal:

```bash
cd /home/unitree/G1D_Grasp_Project
./g1d-hand-board-start
```

This one terminal supervises the project-pinned UDP driver, mapper V5, and
controller V4 as three isolated Foxy processes.

Start the gateway in one additional laptop terminal:

```bash
cd /home/yii/Documents/Codex/2026-09-07/new-chat/outputs/G1D_Grasp_Project
./g1d-hand-laptop-start
```

This opens `/dev/ttyUSB0` and runs only the UDP-to-RS485 gateway. Neither hand
launcher sends PREOPEN, CLOSE, or RELEASE by itself.

The ROS/Conda installations remain external, but the project-specific startup
scripts they invoke are retained under this repository. See
`docs/SCRIPT_INVENTORY.md` for provenance and the remaining live-board export.

Readiness is not based only on process existence. Each of the four supervisors
waits for its own services to report ready. Every child has a separate
timestamped log under `runs/supervisor`.

## Preflight without starting

Both commands support configuration-only checks:

```bash
./g1d-board-start --check
./g1d-laptop-start --check
./g1d-hand-board-start --check
./g1d-hand-laptop-start --check
```

## Planning

After both supervisors are healthy, add the canonical handoff-manual table in
a laptop ROS 2 terminal:

```bash
logon_ros2
source ~/g1_ws/install/setup.bash
python3 ~/下载/add_g1_table_collision.py
```

Require the `TABLE_COLLISION_READY` marker. The same operator-confirmed table
geometry is pinned in `config/active_geometry.json`, so Plan-only can then run:

```bash
./g1dctl plan
```

The plan command re-adds and reads back the same configured table, then runs
only Plan-only into a new timestamped run directory.
