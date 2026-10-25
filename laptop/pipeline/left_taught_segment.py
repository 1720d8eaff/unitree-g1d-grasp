#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Reusable planner for segment 1: current state -> taught motion end."""


def plan_left_taught_segment(node, taught, metadata, measured):
    taught_start = [
        float(value) for value in metadata["taught_start_positions_rad"]
    ]
    node.validate_path_shape(taught, "taught path")
    node.validate_taught_collisions(taught, measured)
    alignment, start_state, planning_time = node.plan_alignment(
        measured, taught_start
    )
    alignment = node.densify_trajectory(alignment, "alignment path")
    alignment_metrics = node.validate_path_shape(
        alignment.joint_trajectory, "alignment path"
    )
    node.validate_path_collisions(
        alignment.joint_trajectory, measured, "对齐轨迹"
    )
    combined = node.combine_alignment_and_taught(alignment, taught)
    return {
        "trajectory": combined,
        "start_state": start_state,
        "alignment_point_count": len(alignment.joint_trajectory.points),
        "taught_point_count": len(taught.points),
        "alignment_planning_time_sec": planning_time,
        "alignment_metrics": alignment_metrics,
    }
