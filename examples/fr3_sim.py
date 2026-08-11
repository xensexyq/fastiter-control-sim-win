#!/usr/bin/env python3
"""FR3 forward/inverse-kinematics and trajectory simulation."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Sequence

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]

import fr3_control_sim as fr3


def _default_description_root() -> Path:
    override = os.environ.get("FRANKA_DESCRIPTION_ROOT")
    if override:
        return Path(override)
    candidates = (
        PROJECT_ROOT / "third_party" / "franka_description",
        PROJECT_ROOT / "franka_description",
        Path("/home/xense/fastiter/franka_description"),
    )
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    return candidates[0]


DEFAULT_DESCRIPTION_ROOT = _default_description_root()


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("demo", "fk", "ik"), default="demo")
    parser.add_argument("--headless", action="store_true", help="Run without MeshCat.")
    parser.add_argument(
        "--no-open-browser",
        action="store_true",
        help="Start MeshCat and print its URL without opening a browser.",
    )
    parser.add_argument("--urdf", type=Path, help="Path to a generated FR3 + hand URDF.")
    parser.add_argument(
        "--description-root",
        type=Path,
        default=DEFAULT_DESCRIPTION_ROOT,
        help="Official franka_description package root (used to resolve meshes).",
    )
    parser.add_argument(
        "--q",
        type=float,
        nargs="+",
        help="FK joint angles in degrees; defaults to a built-in FR3 pose.",
    )
    parser.add_argument(
        "--target",
        type=float,
        nargs="+",
        metavar="VALUE",
        help="IK target: x y z [roll pitch yaw], in meters and radians.",
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--dt", type=float, default=0.02)
    return parser.parse_args()


def _resolve_urdf(requested: Path | None, description_root: Path) -> Path:
    if requested is not None:
        result = requested.expanduser().resolve()
        if not result.is_file():
            raise FileNotFoundError(f"URDF does not exist: {result}")
        return result

    candidates = (
        description_root.expanduser() / "urdfs" / "fr3_franka_hand.urdf",
        PROJECT_ROOT / "models" / "fr3_franka_hand.urdf",
        PROJECT_ROOT / "resources" / "fr3_franka_hand.urdf",
        PROJECT_ROOT / "share" / "fr3_control_sim" / "fr3_franka_hand.urdf",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    searched = "\n  ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        "No generated official FR3 URDF was found. Generate one with\n"
        f"  cd {description_root.expanduser()} && python3 scripts/create_urdf.py fr3\n"
        "or pass --urdf explicitly. Searched:\n  " + searched
    )


def _demo_configuration(home: np.ndarray) -> np.ndarray:
    if home.shape != (7,):
        return home.copy()
    return np.array([0.35, -0.55, 0.25, -2.0, 0.15, 1.65, 0.40], dtype=float)


def _trajectory_array(trajectory: object, nq: int) -> np.ndarray:
    try:
        array = np.asarray(trajectory, dtype=float)
    except (TypeError, ValueError):
        array = np.asarray(
            [point.q if hasattr(point, "q") else point for point in trajectory],
            dtype=float,
        )
    if array.ndim == 1 and array.shape == (nq,):
        array = array.reshape(1, nq)
    if array.ndim != 2 or array.shape[1] != nq:
        raise ValueError(f"trajectory must have shape (N, {nq}), got {array.shape}")
    return array


def _print_pose(label: str, pose: Sequence[Sequence[float]]) -> None:
    matrix = np.asarray(pose, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"forward_kinematics must return a 4x4 pose, got {matrix.shape}")
    xyz = matrix[:3, 3]
    rpy = np.asarray(fr3.rpy_from_pose(matrix), dtype=float)
    print(f"{label} xyz [m]: {np.array2string(xyz, precision=5)}")
    print(f"{label} rpy [rad]: {np.array2string(rpy, precision=5)}")


def _make_target(model: fr3.RobotModel, home: np.ndarray, values: list[float] | None) -> np.ndarray:
    if values is None:
        return np.asarray(model.forward_kinematics(_demo_configuration(home)), dtype=float)
    if len(values) not in (3, 6):
        raise ValueError("--target requires x y z or x y z roll pitch yaw")
    if len(values) == 3:
        home_pose = np.asarray(model.forward_kinematics(home), dtype=float)
        rpy = np.asarray(fr3.rpy_from_pose(home_pose), dtype=float)
    else:
        rpy = np.asarray(values[3:], dtype=float)
    return np.asarray(
        fr3.pose_from_xyz_rpy(np.asarray(values[:3], dtype=float), rpy),
        dtype=float,
    )


def _run_fk(model: fr3.RobotModel, home: np.ndarray, q_values: list[float] | None) -> np.ndarray:
    q = _demo_configuration(home) if q_values is None else np.radians(q_values)
    if q.shape != (model.nq,):
        raise ValueError(f"--q requires {model.nq} joint values, got {q.size}")
    print(f"q [deg]: {np.array2string(np.degrees(q), precision=2)}")
    _print_pose("end effector", model.forward_kinematics(q))
    return q


def _solve_ik(
    model: fr3.RobotModel,
    home: np.ndarray,
    target_values: list[float] | None,
) -> np.ndarray:
    target = _make_target(model, home, target_values)
    _print_pose("target", target)
    result = model.inverse_kinematics(target, home, fr3.IKOptions())
    error = getattr(result, "error", getattr(result, "residual", float("nan")))
    iterations = getattr(result, "iterations", -1)
    print(f"IK success={result.success} iterations={iterations} error={error:.3e}")
    print(f"q [deg]: {np.array2string(np.degrees(result.q), precision=2)}")
    if not result.success:
        raise RuntimeError("inverse kinematics did not converge")
    solved = np.asarray(result.q, dtype=float)
    _print_pose("solved", model.forward_kinematics(solved))
    return solved


def _interactive_fk(
    model: fr3.RobotModel,
    home: np.ndarray,
    visualizer: object | None,
) -> None:
    current = home.copy()
    if visualizer is not None:
        visualizer.update(current)

    print("\nInteractive FK: input joint angles in degrees.")
    joint_labels = " ".join(f"j{index + 1}" for index in range(model.nq))
    print(f"  {joint_labels}    ({model.nq} values)")
    print("  example: 0 -45 0 -135 0 90 45")
    print("  q / quit / exit or an empty line to stop\n")

    while True:
        try:
            line = input("joint angles > ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nexit.")
            break
        if line.lower() in ("", "q", "quit", "exit"):
            print("exit.")
            break

        try:
            q_degrees = np.asarray(
                [float(value) for value in line.split()], dtype=float
            )
        except ValueError:
            print("invalid input\n")
            continue
        if q_degrees.shape != (model.nq,):
            print(f"need {model.nq} values\n")
            continue
        if not np.isfinite(q_degrees).all():
            print("invalid input\n")
            continue

        current = np.radians(q_degrees)
        pose = np.asarray(model.forward_kinematics(current), dtype=float)
        xyz = pose[:3, 3]
        rpy_degrees = np.degrees(
            np.asarray(fr3.rpy_from_pose(pose), dtype=float)
        )
        if visualizer is not None:
            visualizer.update(current)
        print(f"  ee position: {np.array2string(xyz, precision=5)} m")
        print(f"  ee rpy:      {np.array2string(rpy_degrees, precision=3)} deg\n")


def _interactive_ik(
    model: fr3.RobotModel,
    home: np.ndarray,
    visualizer: object | None,
    duration: float,
    dt: float,
) -> None:
    current = home.copy()
    options = fr3.IKOptions()
    if visualizer is not None:
        visualizer.update(current)

    print("\nInteractive IK: input a target pose.")
    print("  x y z                    (meters; keep current orientation)")
    print("  x y z roll pitch yaw     (meters + radians)")
    print("  example: 0.35 0.10 0.45")
    print("  example: 0.35 0.10 0.45 3.1415926 0.0 0.2")
    print("  q / quit / exit or an empty line to stop\n")

    while True:
        current_pose = np.asarray(model.forward_kinematics(current), dtype=float)
        current_xyz = current_pose[:3, 3]
        current_rpy = np.asarray(fr3.rpy_from_pose(current_pose), dtype=float)
        prompt = (
            "target pose "
            f"[xyz={np.array2string(current_xyz, precision=3)}, "
            f"rpy={np.array2string(current_rpy, precision=3)}] > "
        )
        try:
            line = input(prompt).strip()
        except (KeyboardInterrupt, EOFError):
            print("\nexit.")
            break
        if line.lower() in ("", "q", "quit", "exit"):
            print("exit.")
            break

        try:
            values = [float(value) for value in line.split()]
        except ValueError:
            print("invalid input\n")
            continue
        if len(values) not in (3, 6):
            print("need 3 values or 6 values\n")
            continue
        if not np.isfinite(values).all():
            print("invalid input\n")
            continue

        target = _make_target(model, current, values)
        result = model.inverse_kinematics(target, current, options)
        status = "converged" if result.success else "not converged"
        print(
            f"  [{status}] iterations={result.iterations} "
            f"attempts={result.attempts} error={result.error:.3e}"
        )
        print(f"  q [deg]: {np.array2string(np.degrees(result.q), precision=2)}")
        if not result.success:
            print()
            continue

        solved = np.asarray(result.q, dtype=float)
        if visualizer is not None:
            trajectory = _trajectory_array(
                model.minimum_jerk_trajectory(current, solved, duration, dt),
                model.nq,
            )
            tcp_path = [
                np.asarray(model.forward_kinematics(q), dtype=float)[:3, 3]
                for q in trajectory
            ]
            visualizer.draw_path(tcp_path, node_name="interactive_tcp_path")
            visualizer.play(trajectory, dt)
        current = solved
        solved_pose = np.asarray(model.forward_kinematics(current), dtype=float)
        solved_xyz = solved_pose[:3, 3]
        solved_rpy = np.asarray(fr3.rpy_from_pose(solved_pose), dtype=float)
        print(f"  solved xyz [m]:   {np.array2string(solved_xyz, precision=5)}")
        print(f"  solved rpy [rad]: {np.array2string(solved_rpy, precision=5)}\n")


def main() -> None:
    args = _arguments()
    if args.duration <= 0.0 or args.dt <= 0.0:
        raise ValueError("--duration and --dt must be positive")

    description_root = args.description_root.expanduser().resolve()
    urdf_path = _resolve_urdf(args.urdf, description_root)
    model = fr3.RobotModel(str(urdf_path))
    home = np.asarray(model.home_configuration(), dtype=float)
    print(f"URDF: {urdf_path}")
    print(f"end effector: {model.end_effector_frame}")
    print(f"joints ({model.nq}): {', '.join(model.joint_names)}")

    visualizer = None
    if not args.headless:
        from fr3_control_sim.visualizer import Visualizer

        visualizer = Visualizer(
            model,
            urdf_path,
            description_root,
            open_browser=not args.no_open_browser,
        )
        print(f"MeshCat: {visualizer.url}")

    if args.mode == "fk" and args.q is None:
        _interactive_fk(model, home, visualizer)
        return
    if args.mode == "ik" and args.target is None:
        _interactive_ik(model, home, visualizer, args.duration, args.dt)
        return

    if args.mode == "fk":
        q_goal = _run_fk(model, home, args.q)
    else:
        q_goal = _solve_ik(model, home, args.target)

    if args.mode == "demo":
        _print_pose("home", model.forward_kinematics(home))

    if args.mode == "fk":
        if visualizer is not None:
            visualizer.update(q_goal)
        return

    trajectory = _trajectory_array(
        model.minimum_jerk_trajectory(home, q_goal, args.duration, args.dt),
        model.nq,
    )
    print(
        f"trajectory: {len(trajectory)} samples, duration={args.duration:.3f}s, "
        f"dt={args.dt:.3f}s"
    )
    if visualizer is not None:
        tcp_path = [
            np.asarray(model.forward_kinematics(q), dtype=float)[:3, 3]
            for q in trajectory
        ]
        visualizer.draw_path(tcp_path)
        visualizer.play(trajectory, args.dt)
        if args.mode == "demo":
            return_path = _trajectory_array(
                model.minimum_jerk_trajectory(q_goal, home, args.duration, args.dt),
                model.nq,
            )
            visualizer.play(return_path, args.dt)


if __name__ == "__main__":
    main()
