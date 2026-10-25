from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


def load_json(relative):
    return json.loads((ROOT / relative).read_text(encoding="utf-8"))


def sha256(relative):
    return hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()


class ProjectIntegrityTests(unittest.TestCase):
    def test_project_version_and_two_start_commands(self):
        self.assertEqual((ROOT / "VERSION").read_text().strip(), "0.3.14")
        for command in (
            "g1d-board-start",
            "g1d-laptop-start",
            "g1d-hand-board-start",
            "g1d-hand-laptop-start",
        ):
            path = ROOT / command
            self.assertTrue(path.is_file())
            self.assertTrue(path.stat().st_mode & 0o111)

    def test_v2_backend_hash_is_pinned(self):
        config = load_json("config/endpoint_feedback_v2.json")
        self.assertEqual(
            sha256(
                "onboard/trajectory_bridge/g1_d_trajectory_bridge/"
                "trajectory_backend.py"
            ),
            config["expected_source_sha256"],
        )

    def test_taught_trajectory_matches_metadata(self):
        metadata = load_json("data/taught/g1_left_taught_motion.json")
        self.assertEqual(
            sha256("data/taught/g1_left_taught_motion.cdr"),
            metadata["trajectory_sha256"],
        )
        self.assertEqual(metadata["point_count"], 245)

    def test_operator_confirmed_compensated_table_geometry_is_active(self):
        geometry = load_json("config/active_geometry.json")
        self.assertEqual(geometry["column_raise_m"], 0.15)
        self.assertIs(geometry["verified"], True)
        self.assertEqual(
            geometry["profile_id"],
            "column_15cm_existing_tf_operator_confirmed",
        )
        self.assertEqual(
            geometry["work_table"]["center_xyz_m"],
            [0.48, 0.0, -0.18],
        )
        self.assertEqual(geometry["work_table"]["top_z_m"], -0.12)

    def test_real_execution_is_not_exposed(self):
        safety = load_json("config/safety.json")
        self.assertIs(safety["real_execution_exposed_by_g1dctl"], False)
        self.assertIs(safety["automatic_arm_disarm_allowed"], False)
        self.assertEqual(safety["default_mode"], "plan_only")

    def test_dated_base_dependency_was_removed(self):
        source = (
            ROOT
            / "laptop/pipeline/execute_left_taught_then_anygrasp.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("2026-09-01/new-chat/outputs", source)
        self.assertIn(
            'BASE_PATH = HERE / "execute_left_taught_with_auto_start.py"',
            source,
        )

    def test_old_full_plan_is_quarantined(self):
        self.assertTrue(
            (
                ROOT
                / "archive/stale_after_column_raise_15cm/"
                "g1_left_full_taught_to_anygrasp.cdr"
            ).is_file()
        )
        self.assertFalse(
            (ROOT / "data/taught/g1_left_full_taught_to_anygrasp.cdr").exists()
        )

    def test_supervisor_configs_forbid_motion_authority(self):
        for name in ("board", "laptop", "hand_board", "hand_laptop"):
            config = load_json(f"config/processes_{name}.json")
            self.assertIs(config["allows_arm"], False)
            self.assertIs(config["allows_real_execution"], False)
            self.assertGreater(len(config["processes"]), 0)

    def test_environment_setup_does_not_enable_nounset(self):
        for name in ("board", "laptop", "hand_board", "hand_laptop"):
            config = load_json(f"config/processes_{name}.json")
            for process in config["processes"]:
                self.assertNotIn(
                    "set -euo pipefail",
                    process["command"],
                    f"{name}/{process['name']} is incompatible with ROS setup files",
                )

    def test_anygrasp_starts_from_sdk_license_directory(self):
        config = load_json("config/processes_laptop.json")
        anygrasp = next(
            process
            for process in config["processes"]
            if process["name"] == "anygrasp_server"
        )
        command = anygrasp["command"]
        self.assertIn('cd "$ANYGRASP_SDK_ROOT"', command)
        self.assertLess(
            command.index('cd "$ANYGRASP_SDK_ROOT"'),
            command.index("exec python3 -u"),
        )
        self.assertIn("exec python3 -u", command)
        self.assertIn("ANYGRASP_SHOW_VISUALIZATION=1", command)

    def test_anygrasp_sdk_root_is_promoted_ahead_of_gsnet_versions(self):
        source = (
            ROOT / "laptop/anygrasp/server_slanted_stable.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "sys.path[:] = [_sdk_root] + ",
            source,
        )
        self.assertNotIn(
            "if str(ANYGRASP_SDK_ROOT) not in sys.path",
            source,
        )
        self.assertIn("AnyGrasp SDK module:", source)

    def test_laptop_supervisor_uses_collected_startup_chain(self):
        config = load_json("config/processes_laptop.json")
        moveit = next(
            process
            for process in config["processes"]
            if process["name"] == "moveit"
        )
        self.assertIn(
            "@PROJECT_ROOT@/laptop/startup/start_g1_control.sh",
            moveit["command"],
        )
        control = (
            ROOT / "laptop/startup/start_g1_control.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('"$SCRIPT_DIR/start_g1_moveit.sh" &', control)
        self.assertNotIn("bash -ic 'start_g1_moveit'", control)

    def test_two_supervisors_include_complete_rh56h1_service_chain(self):
        main_laptop = load_json("config/processes_laptop.json")
        main_board = load_json("config/processes_board.json")
        laptop = load_json("config/processes_hand_laptop.json")
        board = load_json("config/processes_hand_board.json")
        laptop_names = [item["name"] for item in laptop["processes"]]
        board_names = [item["name"] for item in board["processes"]]
        self.assertFalse(
            any("rh56h1" in item["name"] for item in main_laptop["processes"])
        )
        self.assertFalse(
            any("rh56h1" in item["name"] for item in main_board["processes"])
        )
        self.assertEqual(laptop_names.count("rh56h1_gateway"), 1)
        self.assertEqual(
            board_names,
            ["rh56h1_udp_driver", "rh56h1_mapper", "rh56h1_controller"],
        )
        gateway = laptop["processes"][0]
        self.assertIn(
            "@PROJECT_ROOT@/hand/laptop/rh56h1_gateway_v2.py",
            gateway["command"],
        )
        self.assertEqual(gateway["ready_patterns"], ["UDP 网关已启动"])
        for item in board["processes"]:
            self.assertIn(
                "@PROJECT_ROOT@/onboard/startup/run_rh56h1_service.sh",
                item["command"],
            )

        helper = (
            ROOT / "onboard/startup/run_rh56h1_service.sh"
        ).read_text(encoding="utf-8")
        for marker in (
            "source /opt/ros/foxy/setup.bash",
            "source /home/unitree/ros2_ws/install/setup.bash",
            "source /home/unitree/g1_trajectory_bridge_env.sh",
            "source /home/unitree/cyclonedds_ws/install/setup.bash",
            "rh56h1_virtual_gripper_mapper_v5.py",
            "rh56h1_virtual_gripper_controller_v4.py",
            "rh56h1_udp_driver/rh56h1_udp_driver/driver_node.py",
        ):
            self.assertIn(marker, helper)

    def test_current_runtime_relay_features_are_collected(self):
        relay = (
            ROOT
            / "laptop/bridge/g1_d_moveit_bridge/trajectory_action_relay.py"
        ).read_text(encoding="utf-8")
        for marker in (
            "TIME_SCALE = 5.4",
            "RESAMPLE_MAX_INCREMENT = 0.08",
            "FINAL_SETTLE_TIME_SEC = 3.0",
            "def _resample_trajectory",
        ):
            self.assertIn(marker, relay)

    def test_collected_project_specific_sources_exist(self):
        required = (
            "laptop/startup/start_g1_moveit.sh",
            "laptop/ros_packages/g1_d_moveit_config/launch/real_robot.launch.py",
            "laptop/ros_packages/g1_d_description/g1_d_ros.urdf",
            "onboard/state_snapshot_20260827/unitree_reader.py",
            "onboard/state_snapshot_20260827/state_bridge_node.py",
            "hand/onboard/rh56h1_udp_driver/rh56h1_udp_driver/driver_node.py",
            "tools/collect_onboard_scripts.sh",
            "onboard/startup/start_g1_control.sh",
            "onboard/startup/start_g1_state.sh",
            "onboard/startup/g1_trajectory_bridge_env.sh",
            "onboard/state/unitree_reader.py",
            "onboard/state/g1_d_state_bridge/g1_d_state_bridge/state_bridge_node.py",
            "onboard/height/g1d_height_15cm.cpp",
            "hand/onboard/rh56h1_virtual_gripper_controller_v4.py",
            "hand/onboard/service_interfaces/package.xml",
            "hand/onboard/service_interfaces/CMakeLists.txt",
            "hand/onboard/service_interfaces/msg/SetAngle1.msg",
            "hand/onboard/service_interfaces/msg/GetAngleAct1.msg",
        )
        for relative in required:
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_start_entry_dependency_ledger_covers_all_four_launchers(self):
        ledger = (
            ROOT / "docs/START_ENTRY_DEPENDENCIES.md"
        ).read_text(encoding="utf-8")
        for marker in (
            "g1d-board-start",
            "g1d-laptop-start",
            "g1d-hand-board-start",
            "g1d-hand-laptop-start",
            "tools/process_supervisor.py",
            "hand/onboard/service_interfaces",
            "External runtime dependencies not vendored",
        ):
            self.assertIn(marker, ledger)

    def test_live_board_export_and_promoted_sources_are_hash_pinned(self):
        provenance = load_json("config/onboard_source_provenance.json")
        archive = provenance["archive"]
        expected = {archive["path"]: archive["sha256"]}
        expected.update(provenance["promoted_sources"])
        self.assertEqual(len(provenance["promoted_sources"]), 9)
        for relative, digest in expected.items():
            self.assertEqual(sha256(relative), digest, relative)

    def test_table_verifier_handles_moveit_planning_frame_normalization(self):
        source = (
            ROOT / "laptop/moveit/add_g1_table_collision.py"
        ).read_text(encoding="utf-8")
        for marker in (
            "planning_scene_storage_frame=",
            "lookup_transform(",
            "compose_pose(",
            "position_verify_error_m=",
            "orientation_verify_error_rad=",
        ):
            self.assertIn(marker, source)
        self.assertNotIn(
            'if actual.header.frame_id != self.table["frame_id"]:',
            source,
        )

    def test_final_approach_uses_manual_teach_orientation_and_ranked_paths(self):
        source = (
            ROOT / "laptop/pipeline/anygrasp_segment.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "USE_MANUAL_TEACH_LIVE_TARGET_CALIBRATION = ",
            source,
        )
        self.assertIn(
            "node.allow_verified_orientation_fallback = "
            "PREVIOUS_PLANNER_PREVIEW",
            source,
        )
        self.assertIn(
            'PLANNER_POLICY_VERSION = '
            '"0.3.13-live-position-taught-orientation"',
            source,
        )
        self.assertIn(
            "PREFERRED_GRASP_SEGMENT_DETOUR_RAD = 0.40",
            source,
        )
        self.assertIn("MAX_GRASP_SEGMENT_DETOUR_RAD = 1.20", source)
        self.assertIn("MAX_FRESH_TARGET_PLAN_ATTEMPTS = 1", source)
        self.assertIn("math.radians(-10.0)", source)
        self.assertIn("math.radians(+30.0)", source)
        self.assertIn("FINAL_GRASP_WRIST_YAW_MIN_RAD = math.radians(-5.0)", source)
        self.assertIn("FINAL_GRASP_WRIST_YAW_MAX_RAD = math.radians(+5.0)", source)
        self.assertIn('"FINAL_GRASP"', source)
        self.assertIn("extra_seeds=[item[1] for item in final_ik_candidates]", source)
        self.assertIn("make_wrist_yaw_ik_constraint", source)
        self.assertIn("node.grasp_ik_constraints =", source)
        pipeline_source = (
            ROOT / "laptop/pipeline/execute_left_taught_then_anygrasp.py"
        ).read_text(encoding="utf-8")
        self.assertIn("request.ik_request.constraints =", pipeline_source)
        self.assertIn("self.manual_teach_position_offset_xyz", pipeline_source)
        self.assertIn(
            '"live_anygrasp_position_with_taught_world_offset"',
            pipeline_source,
        )
        self.assertNotIn(
            "target.pose.position.x = float(manual_position[0])",
            pipeline_source,
        )
        self.assertIn(
            "max(stage_score, combined_score)", source
        )
        self.assertIn(
            'overhead_trajectory, "上方接近转移段"', source
        )
        self.assertIn(
            "MAX_OVERHEAD_IK_CANDIDATES = "
            "1 if PREVIOUS_PLANNER_PREVIEW else 4",
            source,
        )
        self.assertIn("TOP-DOWN IK PATH SELECTED", source)

        calibration = load_json(
            "data/references/left_grasp_manual_teach_calibration_20260909.json"
        )
        self.assertEqual(
            calibration["artifact_type"],
            "g1_left_grasp_manual_teach_calibration",
        )
        self.assertAlmostEqual(
            calibration["left_wrist_yaw_endpoint_deg"], 1.0718536890438732
        )
        self.assertTrue(
            calibration["policy"][
                "use_live_anygrasp_position_for_second_segment"
            ]
        )
        self.assertTrue(
            calibration["policy"][
                "use_manual_tcp_orientation_for_second_segment"
            ]
        )
        self.assertFalse(calibration["policy"]["use_relative_tcp_calibration"])
        self.assertFalse(
            calibration["policy"]["use_absolute_torso_tcp_pose"]
        )
        self.assertTrue(
            calibration["policy"]["original_taught_trajectory_is_unchanged"]
        )

    def test_taught_world_offset_tracks_live_anygrasp_position(self):
        calibration = load_json(
            "data/references/left_grasp_manual_teach_calibration_20260909.json"
        )
        source_position = calibration["source_anygrasp_position_m"]
        endpoint = calibration["endpoint_tcp_position_m_diagnostic"]
        offset = calibration["world_tcp_position_offset_m"]
        reconstructed = [a + b for a, b in zip(source_position, offset)]
        for actual, expected in zip(reconstructed, endpoint):
            self.assertAlmostEqual(actual, expected, places=9)
        live_delta = (0.01, -0.02, 0.03)
        moved_live = [a + b for a, b in zip(source_position, live_delta)]
        moved_target = [a + b for a, b in zip(moved_live, offset)]
        target_delta = [a - b for a, b in zip(moved_target, reconstructed)]
        for actual, expected in zip(target_delta, live_delta):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_previous_planner_is_available_only_as_explicit_plan_preview(self):
        controller = (ROOT / "tools/g1dctl.py").read_text(encoding="utf-8")
        planner = (
            ROOT / "laptop/pipeline/anygrasp_segment.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"--previous-preview"', controller)
        self.assertIn(
            'environment["G1D_PREVIOUS_PLANNER_PREVIEW"] = "1"',
            controller,
        )
        self.assertIn("PREVIOUS_PLANNER_PREVIEW", planner)
        self.assertIn("previous_planner_preview", planner)
        self.assertIn(
            "node.manual_teach_position_offset_xyz = (",
            planner,
        )

    def test_saved_full_executor_keeps_volatile_safety_checks(self):
        source = (
            ROOT
            / "laptop/pipeline/execute_full_taught_to_anygrasp_continuous.py"
        ).read_text(encoding="utf-8")
        for marker in (
            '"--execute-saved"',
            '"--accept-known-combined-reversal"',
            "load_saved_full_artifact()",
            "require_saved_start(node, trajectory)",
            "node.validate_path_collisions(",
            "validate_saved_target_current(",
            '"live_anygrasp_position_taught_orientation"',
            "approach.PLANNER_POLICY_VERSION",
            "audit_relay_timing(trajectory)",
            "node.execute_trajectory(",
        ):
            self.assertIn(marker, source)

    def test_saved_grasp_cycle_is_explicit_and_fail_closed(self):
        executor = (
            ROOT
            / "laptop/pipeline/execute_full_taught_to_anygrasp_continuous.py"
        ).read_text(encoding="utf-8")
        replay = (
            ROOT / "laptop/pipeline/execute_left_taught_with_auto_start.py"
        ).read_text(encoding="utf-8")
        hand = (
            ROOT / "laptop/pipeline/rh56h1_gripper_stage.py"
        ).read_text(encoding="utf-8")
        for marker in (
            '"--grasp-cycle"',
            '"--accept-reviewed-target-drift"',
            "--grasp-cycle必须与--execute-saved一起使用",
            "RVIZ_REVIEWED_TARGET_SHIFT_M = 0.015",
            "RVIZ_REVIEWED_TARGET_SHIFT_RAD = math.radians(20.0)",
            "POST_GRASP_LIFT_M = 0.05",
            "GRASP_CLOSE_BEFORE_ENDPOINT_SEC = 1.0",
            "gripper.preopen()",
            "before_end_callback=(gripper.close",
            "plan_post_grasp_lift(",
            "本运行目录已存在抓取后抬升规划，拒绝覆盖",
            "post-grasp lift start acceptance",
            "node.validate_path_collisions(",
            "hand remains closed and no RELEASE was sent",
            "MANUAL_TEACH_TCP_QUATERNION_XYZW",
        ):
            self.assertIn(marker, executor)
        for marker in (
            "before_end_callback=None",
            "callback_error = None",
            'TRAJECTORY_STATUS_TOPIC = "/g1_d/trajectory_status"',
            'str(payload.get("state", "")) == "executing"',
            "TIMED CALLBACK ARMED FROM BOARD START",
            "主轨迹将继续完成",
            "在定时回调触发前结束；未发送回调命令",
        ):
            self.assertIn(marker, replay)
        self.assertIn("self.width_ready = True", hand)
        self.assertIn("if not self.width_ready:", hand)
        self.assertNotIn("/g1_d/set_trajectory_control", executor)

    def test_second_segment_teach_recorder_is_read_only(self):
        recorder = (
            ROOT
            / "laptop/pipeline/record_left_planner_calibration_teach.py"
        ).read_text(encoding="utf-8")
        controller = (ROOT / "tools/g1dctl.py").read_text(encoding="utf-8")
        for forbidden in (
            "create_publisher(",
            "ActionClient(",
            "/g1_d/set_trajectory_control",
            "/virtual_gripper/command",
        ):
            self.assertNotIn(forbidden, recorder)
        for marker in (
            "left_full_motion_for_second_segment_calibration_raw",
            "full_manual_recording_original_taught_path_unchanged",
            "split_at_taught_endpoint",
            "detected_first_to_second_segment_seam",
            "second_segment_joint_metrics",
            '"use_second_segment_metrics_only": True',
            "observed_reversal_detour_rad_by_joint",
            "manual_left_wrist_yaw_endpoint_deg",
            "endpoint_median_tcp",
            'automatic_code_or_gate_changes_applied": False',
        ):
            self.assertIn(marker, recorder)
        self.assertIn('"teach-calibrate"', controller)
        self.assertIn("CALIBRATION_RECORDER", controller)

    def test_supervisor_stops_siblings_when_one_child_exits(self):
        config = {
            "schema_version": 1,
            "role": f"test_{id(self)}",
            "allows_arm": False,
            "allows_real_execution": False,
            "processes": [
                {
                    "name": "short",
                    "startup_grace_sec": 0.05,
                    "command": (
                        f"{sys.executable} -c "
                        "'import time; print(\"ready\"); time.sleep(0.25)'"
                    ),
                },
                {
                    "name": "long",
                    "startup_grace_sec": 0.05,
                    "command": (
                        f"{sys.executable} -c "
                        "'import time; print(\"waiting\"); time.sleep(30)'"
                    ),
                },
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "processes.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/process_supervisor.py"),
                    "--config",
                    str(path),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=8.0,
            )
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("critical process short exited", result.stdout)
        self.assertIn("stopped long", result.stdout)


if __name__ == "__main__":
    unittest.main()
