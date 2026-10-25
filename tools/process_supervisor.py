#!/usr/bin/env python3
"""Foreground supervisor for isolated G1-D service processes.

Every child runs in its own process group with an explicit environment setup.
Output remains visible in the terminal and is also copied to a versioned log.
Ctrl+C performs targeted shutdown of only the process groups created here.
"""

from __future__ import annotations

import argparse
import datetime as dt
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[1]


class SupervisorError(RuntimeError):
    pass


def load_config(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise SupervisorError("unsupported process configuration schema")
    if data.get("allows_arm") is not False:
        raise SupervisorError("process configuration must explicitly forbid ARM")
    if data.get("allows_real_execution") is not False:
        raise SupervisorError(
            "process configuration must explicitly forbid real execution"
        )
    processes = data.get("processes")
    if not isinstance(processes, list) or not processes:
        raise SupervisorError("process configuration has no processes")
    names = []
    for item in processes:
        if not isinstance(item.get("name"), str) or not item["name"]:
            raise SupervisorError("every process needs a non-empty name")
        if not isinstance(item.get("command"), str) or not item["command"]:
            raise SupervisorError(f"process {item['name']} has no command")
        names.append(item["name"])
    if len(names) != len(set(names)):
        raise SupervisorError("process names must be unique")
    return data


def expand_command(command: str) -> str:
    return command.replace("@PROJECT_ROOT@", str(ROOT)).replace(
        "@HOME@", str(Path.home())
    )


class ForegroundSupervisor:
    def __init__(self, config: dict):
        self.config = config
        self.role = config["role"]
        self.children: list[tuple[dict, subprocess.Popen, object]] = []
        self.ready_events: dict[str, threading.Event] = {}
        self.ready_patterns_seen: dict[str, set[str]] = {}
        self.stop_event = threading.Event()
        self.cleanup_started = False
        self.lock_stream = None
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.log_dir = ROOT / "runs" / "supervisor" / f"{stamp}-{self.role}"

    def log(self, message: str):
        print(f"[supervisor:{self.role}] {message}", flush=True)

    def acquire_lock(self):
        lock_path = Path(f"/tmp/g1d_{self.role}_supervisor.lock")
        self.lock_stream = lock_path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(
                self.lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB
            )
        except BlockingIOError as exc:
            raise SupervisorError(
                f"another {self.role} supervisor already owns {lock_path}"
            ) from exc
        self.lock_stream.seek(0)
        self.lock_stream.truncate()
        self.lock_stream.write(f"{os.getpid()}\n")
        self.lock_stream.flush()

    def _stream_output(self, name, process, log_stream):
        if process.stdout is None:
            return
        for line in process.stdout:
            rendered = f"[{name}] {line}"
            print(rendered, end="", flush=True)
            log_stream.write(rendered)
            log_stream.flush()
            spec = next(
                item for item, child, _log in self.children
                if child is process
            )
            patterns = spec.get("ready_patterns", [])
            seen = self.ready_patterns_seen[name]
            for pattern in patterns:
                if pattern in line:
                    seen.add(pattern)
            if patterns and len(seen) == len(patterns):
                self.ready_events[name].set()

    def launch_one(self, spec: dict):
        command = expand_command(spec["command"])
        self.ready_events[spec["name"]] = threading.Event()
        self.ready_patterns_seen[spec["name"]] = set()
        log_path = self.log_dir / f"{spec['name']}.log"
        log_stream = log_path.open("a", encoding="utf-8")
        self.log(f"starting {spec['name']}; log={log_path}")
        process = subprocess.Popen(
            ["/bin/bash", "-lc", command],
            cwd=str(ROOT),
            env={**os.environ, "PYTHONUNBUFFERED": "1"},
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.children.append((spec, process, log_stream))
        thread = threading.Thread(
            target=self._stream_output,
            args=(spec["name"], process, log_stream),
            daemon=True,
        )
        thread.start()
        grace = float(spec.get("startup_grace_sec", 1.0))
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            code = process.poll()
            if code is not None:
                raise SupervisorError(
                    f"{spec['name']} exited during startup with code {code}"
                )
            time.sleep(0.05)
        patterns = spec.get("ready_patterns", [])
        if patterns:
            ready_deadline = time.monotonic() + float(
                spec.get("ready_timeout_sec", 30.0)
            )
            while time.monotonic() < ready_deadline:
                if self.ready_events[spec["name"]].wait(timeout=0.1):
                    self.log(f"ready {spec['name']}: {patterns}")
                    break
                code = process.poll()
                if code is not None:
                    raise SupervisorError(
                        f"{spec['name']} exited before ready with code {code}"
                    )
            else:
                missing = [
                    value for value in patterns
                    if value not in self.ready_patterns_seen[spec["name"]]
                ]
                raise SupervisorError(
                    f"{spec['name']} readiness timeout; missing={missing}"
                )

    def start(self):
        self.acquire_lock()
        self.log_dir.mkdir(parents=True, exist_ok=False)
        self.log("services only; ARM and real execution are not performed")
        for spec in self.config["processes"]:
            self.launch_one(spec)
        self.log("all configured processes started; Ctrl+C stops this stack")

    def monitor(self):
        while not self.stop_event.is_set():
            for spec, process, _log in self.children:
                code = process.poll()
                if code is not None:
                    raise SupervisorError(
                        f"critical process {spec['name']} exited with code {code}"
                    )
            time.sleep(0.2)

    @staticmethod
    def _signal_group(process, signum):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                pass

    def cleanup(self):
        if self.cleanup_started:
            return
        self.cleanup_started = True
        self.stop_event.set()
        self.log("stopping only child process groups created by this supervisor")
        for _spec, process, _log in reversed(self.children):
            self._signal_group(process, signal.SIGINT)
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if all(process.poll() is not None for _, process, _ in self.children):
                break
            time.sleep(0.1)
        for _spec, process, _log in reversed(self.children):
            self._signal_group(process, signal.SIGTERM)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if all(process.poll() is not None for _, process, _ in self.children):
                break
            time.sleep(0.1)
        for _spec, process, _log in reversed(self.children):
            self._signal_group(process, signal.SIGKILL)
        for spec, process, log_stream in self.children:
            try:
                code = process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                code = "unknown"
            log_stream.close()
            self.log(f"stopped {spec['name']} code={code}")


def print_check(config: dict, path: Path):
    print(f"CONFIG_OK role={config['role']} file={path}")
    print("ARM_ALLOWED=false")
    print("REAL_EXECUTION_ALLOWED=false")
    for item in config["processes"]:
        print(f"PROCESS {item['name']}: {expand_command(item['command'])}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        config = load_config(args.config)
        if args.check:
            print_check(config, args.config)
            return 0
        supervisor = ForegroundSupervisor(config)

        def request_stop(_signum, _frame):
            supervisor.stop_event.set()

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        try:
            supervisor.start()
            supervisor.monitor()
            return 0
        finally:
            supervisor.cleanup()
    except SupervisorError as exc:
        print(f"SUPERVISOR_ERROR: {exc}", file=sys.stderr)
        try:
            if "supervisor" in locals():
                supervisor.cleanup()
        except Exception as cleanup_exc:
            print(f"CLEANUP_ERROR: {cleanup_exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
