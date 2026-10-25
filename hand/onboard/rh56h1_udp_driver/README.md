# rh56h1_udp_driver

ROS 2 Foxy driver for RH56H1 through a laptop UDP-to-RS485 gateway.

Default gateway:
- IP: 192.168.123.222
- UDP port: 5005
- Hand ID: 1

Topics:
- Publishes `angle_data` (`service_interfaces/msg/GetAngleAct1`)
- Publishes `force_data` (`service_interfaces/msg/GetForceAct1`)
- Subscribes `set_angle_data` (`service_interfaces/msg/SetAngle1`)
- Subscribes `set_force_data` (`service_interfaces/msg/SetForce1`)
- Subscribes `set_speed_data` (`service_interfaces/msg/SetSpeed1`)

Important:
- Single-finger commands are sent as single-register writes.
- The driver deliberately does not use 6-axis `-1` placeholder group writes.
