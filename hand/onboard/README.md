# RH56H1 board-side collection status

The controller V4, mapper V5, and complete `rh56h1_udp_driver` ROS package are
collected here and were hash-checked against the 2026-09-07 live-board export.
The laptop UDP gateway is under `hand/laptop`.

The three board-side hand services are started and monitored together by the
separate `./g1d-hand-board-start` foreground supervisor. They are deliberately
not part of `./g1d-board-start`. The helper under `onboard/startup` keeps each
service in the board's Foxy/Cyclone environment and runs the pinned driver
source from this project. Starting these services does not send a hand command;
PREOPEN/CLOSE remain exclusive to the explicit reviewed grasp cycle.
