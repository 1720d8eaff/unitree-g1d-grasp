# Architecture and process boundaries

The project is one repository, not one process.  The following boundaries are
intentional and must remain visible to maintainers.

## Board

The board uses Ubuntu 20.04 and ROS 2 Foxy.  The trajectory frontend is a pure
ROS process; the 500 Hz trajectory backend is a pure Unitree DDS process.  They
communicate through a local Unix-domain socket.  Combining them into one Python
process previously caused DDS incompatibility and allocation failures.

The D435 pipeline runs in the separate `tv_jetson` Conda environment.  It is
collected under `onboard/vision` but is not imported into the trajectory
backend.

## Laptop

The laptop uses Ubuntu 22.04 and ROS 2 Humble for MoveIt, the trajectory relay,
the candidate selector, and the unified planner.  The AnyGrasp network runs in
its own Conda environment.  `g1dctl terminals` prints the current environment
boundary for each process.

## Safety boundary

`g1dctl` may check files, activate a measured geometry profile, add a verified
collision object, and run Plan-only.  Version 1 deliberately refuses real
execution.  The board remains DISARMED until a separate onsite decision.

## Data flow

```text
D435 pipeline -> AnyGrasp server -> candidate-A selector
       -> unified MoveIt Plan-only pipeline -> reviewed run artifact
       -> separately authorized arm-only execution -> endpoint V2 feedback
```

RH56H1 remains outside the default path until its full communication chain and
the endpoint acceptance gate are both validated.

