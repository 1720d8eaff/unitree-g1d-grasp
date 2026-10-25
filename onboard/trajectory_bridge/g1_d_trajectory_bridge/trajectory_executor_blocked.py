#!/usr/bin/env python3

"""Blocked legacy single-process entry point."""


def main() -> None:
    raise SystemExit(
        "已禁用旧的单进程 trajectory_executor：同进程混用 ROS DDS 与 "
        "Unitree CycloneDDS 已验证不稳定。请分别启动 trajectory_backend "
        "和 trajectory_frontend。"
    )


if __name__ == "__main__":
    main()
