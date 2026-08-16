#!/usr/bin/env python3
"""Pybind-level FK/IK smoke test; all kinematics execute in C++."""

from __future__ import annotations

from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

import fr3_control_sim as fr3


def _test_cad_urdf() -> None:
    urdf = PROJECT_ROOT / "models" / "URDF" / "URDF.urdf"
    model = fr3.RobotModel(str(urdf))
    assert model.nq == 7
    assert model.end_effector_frame == "link_7"
    assert model.joint_names == [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
        "joint_7",
    ]
    home = np.asarray(model.home_configuration(), dtype=float)
    expected = np.radians([0.0, -45.0, 0.0, 135.0, 0.0, -90.0, -45.0])
    assert np.allclose(home, expected)

    target_q = home + np.radians([4.0, 3.0, -4.0, -3.0, 3.0, 2.0, -3.0])
    target = np.asarray(model.forward_kinematics(target_q), dtype=float)
    result = model.inverse_kinematics(target, home, fr3.IKOptions())
    assert result.success, result
    recovered = np.asarray(model.forward_kinematics(result.q), dtype=float)
    assert np.linalg.norm(recovered[:3, 3] - target[:3, 3]) < 1e-5

    # The CAD URDF deliberately uses paths relative to its own directory.
    from fr3_control_sim.visualizer import _parse_visuals

    visuals = _parse_visuals(urdf, None)
    assert len(visuals) == 8
    assert all(visual.geometry_kind == "mesh" for visual in visuals)
    assert all(Path(visual.geometry_data).is_file() for visual in visuals)


def main() -> None:
    model = fr3.RobotModel(str(PROJECT_ROOT / "models" / "fr3_franka_hand.urdf"))
    assert model.nq == 7
    assert model.joint_names == [f"fr3_joint{index}" for index in range(1, 8)]

    home = np.asarray(model.home_configuration(), dtype=float)
    target_q = home + np.array([0.20, 0.10, -0.15, 0.10, 0.12, -0.10, -0.20])
    target = np.asarray(model.forward_kinematics(target_q), dtype=float)
    result = model.inverse_kinematics(target, home, fr3.IKOptions())
    assert result.success, result

    recovered = np.asarray(model.forward_kinematics(result.q), dtype=float)
    position_error = float(np.linalg.norm(recovered[:3, 3] - target[:3, 3]))
    assert position_error < 1e-5, position_error

    trajectory = np.asarray(
        model.minimum_jerk_trajectory(home, result.q, 1.0, 0.02), dtype=float
    )
    assert trajectory.shape == (51, 7)
    assert np.allclose(trajectory[0], home)
    assert np.allclose(trajectory[-1], result.q)

    skewed_seed = np.radians(
        [-27.75, -48.05, 17.44, -134.54, 12.89, 87.89, 29.59]
    )
    near_home_target = np.asarray(
        fr3.pose_from_xyz_rpy(
            np.array([0.3069, 0.0, 0.4869]),
            np.array([-3.1416, 0.0, 0.0]),
        ),
        dtype=float,
    )
    posture_options = fr3.IKOptions()
    posture_options.max_retries = 0
    posture_options.posture_gain = 0.1
    posture_result = model.inverse_kinematics(
        near_home_target, skewed_seed, posture_options
    )
    assert posture_result.success, posture_result
    posture_distance = float(np.linalg.norm(posture_result.q - home))
    assert posture_distance < 1e-3, posture_distance

    _test_cad_urdf()

    print("Python/pybind smoke test passed")
    print(f"  IK residual: {result.error:.3e}")
    print(f"  position round-trip error: {position_error:.3e} m")
    print(f"  null-space home distance: {posture_distance:.3e} rad")
    print("  CAD URDF load/FK/IK/STL paths: passed")


if __name__ == "__main__":
    main()
