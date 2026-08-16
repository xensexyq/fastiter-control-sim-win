#!/usr/bin/env python3
"""Qt slider control panel for a 7-DoF Pinocchio C++ simulation."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time
from typing import Sequence

import numpy as np

import fr3_control_sim as fr3

PROJECT_ROOT = Path(__file__).resolve().parents[1]
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
# Edit this value to change the default null-space home-posture constraint.
# Set it to 0.0 to disable the constraint.  It can also be overridden with
# the --posture-gain command-line option or the IK-tab spin box.
DEFAULT_POSTURE_GAIN = float(fr3.IKOptions().posture_gain)
DEFAULT_IK_UPDATE_HZ = 30.0
DEFAULT_RENDER_HZ = 60.0
DEFAULT_SMOOTH_TIME = 0.12

try:
    from PySide6.QtCore import QSignalBlocker, Qt, QTimer, QUrl, Signal
    from PySide6.QtGui import QDesktopServices
    from PySide6.QtWidgets import (
        QApplication,
        QDoubleSpinBox,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSlider,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )
except ModuleNotFoundError as exc:
    raise SystemExit(
        "PySide6 is required for fr3_sim_qt.py. Install it with:\n"
        "  mamba install -n fr3sim -c conda-forge pyside6\n"
        "or update the environment with:\n"
        "  mamba env update -n fr3sim -f environment.yml"
    ) from exc


def _resolve_urdf(requested: Path | None, description_root: Path) -> Path:
    if requested is not None:
        result = requested.expanduser().resolve()
        if not result.is_file():
            raise FileNotFoundError(f"URDF does not exist: {result}")
        return result

    candidates = (
        description_root.expanduser() / "urdfs" / "fr3_franka_hand.urdf",
        PROJECT_ROOT / "models" / "fr3_franka_hand.urdf",
        PROJECT_ROOT / "models" / "URDF" / "URDF.urdf",
        PROJECT_ROOT / "resources" / "fr3_franka_hand.urdf",
        PROJECT_ROOT / "share" / "fr3_control_sim" / "fr3_franka_hand.urdf",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    searched = "\n  ".join(str(path) for path in candidates)
    raise FileNotFoundError("No supported 7-DoF URDF was found. Searched:\n  " + searched)


class FloatControl(QWidget):
    """A floating-point value controlled by a slider and a spin box."""

    value_changed = Signal(float)

    def __init__(
        self,
        label: str,
        minimum: float,
        maximum: float,
        value: float,
        *,
        decimals: int,
        single_step: float,
        suffix: str,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        if not minimum < maximum:
            raise ValueError(f"invalid range for {label}: [{minimum}, {maximum}]")

        self._minimum = float(minimum)
        self._maximum = float(maximum)
        self._slider_steps = 20_000

        self.name_label = QLabel(label)
        self.name_label.setMinimumWidth(105)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, self._slider_steps)
        self.slider.setTracking(True)

        self.spin_box = QDoubleSpinBox()
        self.spin_box.setRange(self._minimum, self._maximum)
        self.spin_box.setDecimals(decimals)
        self.spin_box.setSingleStep(single_step)
        self.spin_box.setSuffix(suffix)
        self.spin_box.setKeyboardTracking(True)
        self.spin_box.setMinimumWidth(135)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.addWidget(self.name_label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin_box)

        self.slider.valueChanged.connect(self._on_slider_changed)
        self.spin_box.valueChanged.connect(self._on_spin_changed)
        self.set_value(value)

    def _position_from_value(self, value: float) -> int:
        ratio = (value - self._minimum) / (self._maximum - self._minimum)
        return int(round(np.clip(ratio, 0.0, 1.0) * self._slider_steps))

    def _value_from_position(self, position: int) -> float:
        ratio = float(position) / self._slider_steps
        return self._minimum + ratio * (self._maximum - self._minimum)

    def _on_slider_changed(self, position: int) -> None:
        value = self._value_from_position(position)
        blocker = QSignalBlocker(self.spin_box)
        self.spin_box.setValue(value)
        del blocker
        self.value_changed.emit(value)

    def _on_spin_changed(self, value: float) -> None:
        blocker = QSignalBlocker(self.slider)
        self.slider.setValue(self._position_from_value(value))
        del blocker
        self.value_changed.emit(float(value))

    def value(self) -> float:
        return float(self.spin_box.value())

    def set_value(self, value: float, *, emit: bool = False) -> None:
        clamped = float(np.clip(value, self._minimum, self._maximum))
        slider_blocker = QSignalBlocker(self.slider)
        spin_blocker = QSignalBlocker(self.spin_box)
        self.slider.setValue(self._position_from_value(clamped))
        self.spin_box.setValue(clamped)
        del slider_blocker
        del spin_blocker
        if emit:
            self.value_changed.emit(clamped)


class Fr3SimWindow(QMainWindow):
    """FK and IK slider controls backed by the C++ ``RobotModel``."""

    def __init__(
        self,
        model: fr3.RobotModel,
        visualizer: object | None,
        urdf_path: Path,
        *,
        initial_mode: str = "fk",
        posture_gain: float = DEFAULT_POSTURE_GAIN,
        ik_update_hz: float = DEFAULT_IK_UPDATE_HZ,
        render_hz: float = DEFAULT_RENDER_HZ,
        smooth_time: float = DEFAULT_SMOOTH_TIME,
    ) -> None:
        super().__init__()
        self.model = model
        self.visualizer = visualizer
        self.urdf_path = urdf_path
        self.home_q = np.asarray(model.home_configuration(), dtype=float)
        self.current_q = self.home_q.copy()
        self.ik_options = fr3.IKOptions()
        if not np.isfinite(posture_gain) or posture_gain < 0.0:
            raise ValueError("--posture-gain must be a finite non-negative number")
        if not np.isfinite(ik_update_hz) or ik_update_hz <= 0.0:
            raise ValueError("--ik-update-hz must be a finite positive number")
        if not np.isfinite(render_hz) or render_hz <= 0.0:
            raise ValueError("--render-hz must be a finite positive number")
        if not np.isfinite(smooth_time) or smooth_time < 0.0:
            raise ValueError("--smooth-time must be a finite non-negative number")
        self.ik_options.posture_gain = float(posture_gain)
        self.ik_update_hz = float(ik_update_hz)
        self.render_hz = float(render_hz)
        self.smooth_time = float(smooth_time)
        self._syncing_controls = False
        self._ik_pending = False
        self._animation_start_q = self.current_q.copy()
        self._animation_goal_q = self.current_q.copy()
        self._animation_started_at = time.monotonic()

        self.fk_timer = QTimer(self)
        self.fk_timer.setSingleShot(True)
        self.fk_timer.setInterval(20)
        self.fk_timer.timeout.connect(self._apply_fk)

        self.ik_timer = QTimer(self)
        self.ik_timer.setSingleShot(True)
        # Do not restart this timer on every slider event.  It samples the
        # newest target at a bounded rate, so dragging continuously still moves.
        self.ik_timer.setInterval(max(1, round(1000.0 / self.ik_update_hz)))
        self.ik_timer.timeout.connect(self._solve_ik)

        self.render_timer = QTimer(self)
        self.render_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.render_timer.setInterval(max(1, round(1000.0 / self.render_hz)))
        self.render_timer.timeout.connect(self._render_animation_frame)

        self.setWindowTitle("7-DoF Pinocchio FK / IK Control")
        self.resize(100, 300)
        self._build_ui()
        self._reset_home()
        self.tabs.setCurrentIndex(0 if initial_mode == "fk" else 1)
        self.render_timer.start()

    def _build_ui(self) -> None:
        central = QWidget()
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(16, 14, 16, 14)
        root_layout.setSpacing(10)

        title = QLabel("7-DoF Pinocchio C++ FK / IK")
        title.setStyleSheet("font-size: 20px; font-weight: 600;")
        root_layout.addWidget(title)
        root_layout.addWidget(QLabel(f"URDF: {self.urdf_path}"))

        meshcat_row = QHBoxLayout()
        if self.visualizer is None:
            self.meshcat_label = QLabel("MeshCat: disabled")
            self.open_meshcat_button = QPushButton("Open MeshCat")
            self.open_meshcat_button.setEnabled(False)
        else:
            self.meshcat_url = str(self.visualizer.url)
            self.meshcat_label = QLabel(f"MeshCat: {self.meshcat_url}")
            self.open_meshcat_button = QPushButton("Open MeshCat")
            self.open_meshcat_button.clicked.connect(self._open_meshcat)
        meshcat_row.addWidget(self.meshcat_label, 1)
        meshcat_row.addWidget(self.open_meshcat_button)
        root_layout.addLayout(meshcat_row)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_fk_tab(), "FK - Joint sliders")
        self.tabs.addTab(self._build_ik_tab(), "IK - XYZ / RPY sliders")
        self.tabs.currentChanged.connect(self._on_tab_changed)
        root_layout.addWidget(self.tabs, 1)

        buttons = QHBoxLayout()
        home_button = QPushButton("Reset Home")
        home_button.clicked.connect(self._reset_home)
        buttons.addStretch(1)
        buttons.addWidget(home_button)
        root_layout.addLayout(buttons)

        self.setCentralWidget(central)

    def _build_fk_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        description = QLabel(
            "Drag joint1 ... joint7 in degrees. Joint ranges come from the C++ model."
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        group = QGroupBox("Joint angles")
        group_layout = QVBoxLayout(group)
        limits = np.asarray(self.model.joint_limits, dtype=float)
        home_degrees = np.degrees(self.home_q)
        self.fk_controls: list[FloatControl] = []
        for index, (joint_name, limit, value) in enumerate(
            zip(self.model.joint_names, limits, home_degrees, strict=True)
        ):
            control = FloatControl(
                f"joint{index + 1} ({joint_name})",
                math.degrees(float(limit[0])),
                math.degrees(float(limit[1])),
                float(value),
                decimals=2,
                single_step=0.1,
                suffix=" deg",
            )
            control.value_changed.connect(self._schedule_fk)
            self.fk_controls.append(control)
            group_layout.addWidget(control)
        layout.addWidget(group)

        self.fk_status = QLabel()
        self.fk_status.setWordWrap(True)
        self.fk_status.setStyleSheet("font-family: monospace;")
        layout.addWidget(self.fk_status)
        layout.addStretch(1)
        return page

    def _build_ik_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        description = QLabel(
            "Drag x/y/z in meters and roll/pitch/yaw in radians. "
            "IK tracks the latest slider target at a fixed rate, while the "
            "displayed joint motion is smoothed independently. The FK sliders "
            "show the latest solved target, not an in-between display frame."
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        group = QGroupBox("Target pose")
        group_layout = QVBoxLayout(group)
        self.ik_controls: dict[str, FloatControl] = {}
        specifications = (
            ("x", -0.9, 0.9, 4, 0.001, " m"),
            ("y", -0.9, 0.9, 4, 0.001, " m"),
            ("z", 0.0, 1.2, 4, 0.001, " m"),
            ("roll", -math.pi, math.pi, 4, 0.01, " rad"),
            ("pitch", -math.pi, math.pi, 4, 0.01, " rad"),
            ("yaw", -math.pi, math.pi, 4, 0.01, " rad"),
        )
        for name, minimum, maximum, decimals, step, suffix in specifications:
            control = FloatControl(
                name,
                minimum,
                maximum,
                0.0,
                decimals=decimals,
                single_step=step,
                suffix=suffix,
            )
            control.value_changed.connect(self._schedule_ik)
            self.ik_controls[name] = control
            group_layout.addWidget(control)
        layout.addWidget(group)

        options_group = QGroupBox("IK options")
        options_layout = QHBoxLayout(options_group)
        options_layout.addWidget(QLabel("posture_gain (null-space):"))
        self.posture_gain_spin_box = QDoubleSpinBox()
        self.posture_gain_spin_box.setRange(0.0, 10.0)
        self.posture_gain_spin_box.setDecimals(3)
        self.posture_gain_spin_box.setSingleStep(0.01)
        self.posture_gain_spin_box.setValue(float(self.ik_options.posture_gain))
        self.posture_gain_spin_box.setToolTip(
            "Home-posture attraction in the Jacobian null space; 0 disables it."
        )
        self.posture_gain_spin_box.valueChanged.connect(
            self._on_posture_gain_changed
        )
        options_layout.addWidget(self.posture_gain_spin_box)
        options_layout.addStretch(1)
        layout.addWidget(options_group)

        ik_buttons = QHBoxLayout()
        current_target_button = QPushButton("Set target from current pose")
        current_target_button.clicked.connect(self._sync_ik_target_from_current)
        solve_button = QPushButton("Solve IK now")
        solve_button.clicked.connect(self._solve_ik_now)
        ik_buttons.addWidget(current_target_button)
        ik_buttons.addStretch(1)
        ik_buttons.addWidget(solve_button)
        layout.addLayout(ik_buttons)

        self.ik_status = QLabel()
        self.ik_status.setWordWrap(True)
        self.ik_status.setStyleSheet("font-family: monospace;")
        layout.addWidget(self.ik_status)
        layout.addStretch(1)
        return page

    def _on_posture_gain_changed(self, value: float) -> None:
        """Apply a new null-space gain and re-solve the current IK target."""
        if self._syncing_controls:
            return
        self.ik_options.posture_gain = float(value)
        self.ik_status.setText(
            f"posture_gain = {self.ik_options.posture_gain:.3f}\n"
            "IK option updated; solving..."
        )
        self.ik_status.setStyleSheet("font-family: monospace; color: #b36b00;")
        if self.tabs.currentIndex() == 1:
            self._schedule_ik(value)

    @staticmethod
    def _pose_text(pose: Sequence[Sequence[float]]) -> str:
        matrix = np.asarray(pose, dtype=float)
        xyz = matrix[:3, 3]
        rpy = np.asarray(fr3.rpy_from_pose(matrix), dtype=float)
        return (
            f"xyz [m]   = {np.array2string(xyz, precision=5)}\n"
            f"rpy [rad] = {np.array2string(rpy, precision=5)}"
        )

    def _update_visualizer(self, q: np.ndarray) -> None:
        if self.visualizer is not None:
            self.visualizer.update(q)

    def _sample_animation(self, now: float | None = None) -> np.ndarray:
        if self.smooth_time <= 0.0:
            return self._animation_goal_q.copy()
        elapsed = (
            (time.monotonic() if now is None else now) - self._animation_started_at
        )
        progress = float(np.clip(elapsed / self.smooth_time, 0.0, 1.0))
        progress2 = progress * progress
        progress3 = progress2 * progress
        blend = (
            10.0 * progress3
            - 15.0 * progress3 * progress
            + 6.0 * progress3 * progress2
        )
        return self._animation_start_q + blend * (
            self._animation_goal_q - self._animation_start_q
        )

    def _set_display_goal(self, q: np.ndarray, *, immediate: bool = False) -> None:
        goal = np.asarray(q, dtype=float)
        if immediate or self.smooth_time <= 0.0:
            self.current_q = goal.copy()
            self._animation_start_q = goal.copy()
            self._animation_goal_q = goal.copy()
            self._animation_started_at = time.monotonic()
            self._update_visualizer(goal)
            return

        now = time.monotonic()
        # Retarget from the pose currently on screen to avoid visible jumps.
        self.current_q = self._sample_animation(now)
        self._animation_start_q = self.current_q.copy()
        self._animation_goal_q = goal.copy()
        self._animation_started_at = now

    def _render_animation_frame(self) -> None:
        if np.array_equal(self.current_q, self._animation_goal_q):
            return
        now = time.monotonic()
        if (
            self.smooth_time <= 0.0
            or now - self._animation_started_at >= self.smooth_time
        ):
            self.current_q = self._animation_goal_q.copy()
        else:
            self.current_q = self._sample_animation(now)
        self._update_visualizer(self.current_q)

    def _schedule_fk(self, _value: float) -> None:
        if self._syncing_controls:
            return
        self.ik_timer.stop()
        self._ik_pending = False
        self.fk_timer.start()

    def _schedule_ik(self, _value: float) -> None:
        if self._syncing_controls:
            return
        self.fk_timer.stop()
        self._ik_pending = True
        self.ik_status.setText("Solving IK...")
        self.ik_status.setStyleSheet("font-family: monospace; color: #b36b00;")
        if not self.ik_timer.isActive():
            self.ik_timer.start()

    def _joint_values_radians(self) -> np.ndarray:
        return np.radians([control.value() for control in self.fk_controls])

    def _target_pose(self) -> np.ndarray:
        xyz = np.array(
            [self.ik_controls[name].value() for name in ("x", "y", "z")],
            dtype=float,
        )
        rpy = np.array(
            [
                self.ik_controls[name].value()
                for name in ("roll", "pitch", "yaw")
            ],
            dtype=float,
        )
        return np.asarray(fr3.pose_from_xyz_rpy(xyz, rpy), dtype=float)

    def _set_fk_controls(self, q: np.ndarray) -> None:
        self._syncing_controls = True
        try:
            for control, value in zip(
                self.fk_controls, np.degrees(q), strict=True
            ):
                control.set_value(float(value))
        finally:
            self._syncing_controls = False

    def _set_ik_controls_from_pose(self, pose: np.ndarray) -> None:
        xyz = pose[:3, 3]
        rpy = np.asarray(fr3.rpy_from_pose(pose), dtype=float)
        self._syncing_controls = True
        try:
            for name, value in zip(("x", "y", "z"), xyz, strict=True):
                self.ik_controls[name].set_value(float(value))
            for name, value in zip(("roll", "pitch", "yaw"), rpy, strict=True):
                self.ik_controls[name].set_value(float(value))
        finally:
            self._syncing_controls = False

    def _apply_fk(self) -> None:
        try:
            q = self._joint_values_radians()
            pose = np.asarray(self.model.forward_kinematics(q), dtype=float)
            self._set_display_goal(q, immediate=True)
            self.fk_status.setText(
                f"q [deg] = {np.array2string(np.degrees(q), precision=2)}\n"
                + self._pose_text(pose)
            )
            self.fk_status.setStyleSheet("font-family: monospace; color: #1b7f2a;")
        except Exception as exc:  # keep the GUI responsive on invalid input
            self.fk_status.setText(f"FK failed: {exc}")
            self.fk_status.setStyleSheet("font-family: monospace; color: #b00020;")

    def _solve_ik(self) -> None:
        self.ik_timer.stop()
        self._ik_pending = False
        try:
            target = self._target_pose()
            result = self.model.inverse_kinematics(
                target, self._animation_goal_q, self.ik_options
            )
            if not result.success:
                self.ik_status.setText(
                    "IK not converged\n"
                    f"posture_gain={self.ik_options.posture_gain:.3f}\n"
                    f"iterations={result.iterations} attempts={result.attempts} "
                    f"error={result.error:.3e}\n"
                    f"position_error={result.position_error:.3e} "
                    f"orientation_error={result.orientation_error:.3e}"
                )
                self.ik_status.setStyleSheet(
                    "font-family: monospace; color: #b00020;"
                )
                return

            solved = np.asarray(result.q, dtype=float)
            self._set_fk_controls(solved)
            self._set_display_goal(solved)
            solved_pose = np.asarray(
                self.model.forward_kinematics(solved), dtype=float
            )
            self.ik_status.setText(
                "IK converged\n"
                f"posture_gain={self.ik_options.posture_gain:.3f}\n"
                f"iterations={result.iterations} attempts={result.attempts} "
                f"error={result.error:.3e}\n"
                f"q [deg] = {np.array2string(np.degrees(solved), precision=2)}\n"
                + self._pose_text(solved_pose)
            )
            self.ik_status.setStyleSheet(
                "font-family: monospace; color: #1b7f2a;"
            )
        except Exception as exc:  # keep the GUI responsive on invalid targets
            self.ik_status.setText(f"IK failed: {exc}")
            self.ik_status.setStyleSheet("font-family: monospace; color: #b00020;")
        finally:
            if self._ik_pending and self.tabs.currentIndex() == 1:
                self.ik_timer.start()

    def _solve_ik_now(self) -> None:
        self._ik_pending = True
        self._solve_ik()

    def _sync_ik_target_from_current(self) -> None:
        self.ik_timer.stop()
        self._ik_pending = False
        displayed = self._sample_animation()
        self._set_display_goal(displayed, immediate=True)
        pose = np.asarray(
            self.model.forward_kinematics(self.current_q), dtype=float
        )
        self._set_ik_controls_from_pose(pose)
        self.ik_status.setText(
            "IK target synchronized with the current pose.\n"
            + self._pose_text(pose)
        )
        self.ik_status.setStyleSheet("font-family: monospace;")

    def _reset_home(self) -> None:
        self.fk_timer.stop()
        self.ik_timer.stop()
        self._ik_pending = False
        self._set_display_goal(self.home_q, immediate=True)
        self._set_fk_controls(self.current_q)
        pose = np.asarray(
            self.model.forward_kinematics(self.current_q), dtype=float
        )
        self._set_ik_controls_from_pose(pose)
        self.fk_status.setText(
            f"q [deg] = {np.array2string(np.degrees(self.current_q), precision=2)}\n"
            + self._pose_text(pose)
        )
        self.fk_status.setStyleSheet("font-family: monospace; color: #1b7f2a;")
        self.ik_status.setText(
            "IK target initialized from home.\n" + self._pose_text(pose)
        )
        self.ik_status.setStyleSheet("font-family: monospace;")

    def _on_tab_changed(self, index: int) -> None:
        if index == 0:
            self.ik_timer.stop()
            self._ik_pending = False
            self._set_fk_controls(self.current_q)
            self._apply_fk()
        else:
            self.fk_timer.stop()
            self._sync_ik_target_from_current()

    def _open_meshcat(self) -> None:
        if self.visualizer is not None:
            QDesktopServices.openUrl(QUrl(self.meshcat_url))


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("fk", "ik"), default="fk")
    parser.add_argument("--urdf", type=Path, help="Path to a 7-DoF robot URDF.")
    parser.add_argument(
        "--end-effector",
        default="",
        help="End-effector frame name; empty selects fr3_hand_tcp or link_7 automatically.",
    )
    parser.add_argument(
        "--description-root",
        type=Path,
        default=DEFAULT_DESCRIPTION_ROOT,
        help="franka_description package root used to resolve MeshCat meshes.",
    )
    parser.add_argument(
        "--no-open-browser",
        action="store_true",
        help="Start MeshCat without opening its browser page automatically.",
    )
    parser.add_argument(
        "--no-meshcat",
        action="store_true",
        help="Run the Qt control panel without MeshCat visualization.",
    )
    parser.add_argument(
        "--posture-gain",
        type=float,
        default=DEFAULT_POSTURE_GAIN,
        help=(
            f"Null-space home-posture gain for IK (default: {DEFAULT_POSTURE_GAIN:g}; "
            "0 disables the constraint)."
        ),
    )
    parser.add_argument(
        "--ik-update-hz",
        type=float,
        default=DEFAULT_IK_UPDATE_HZ,
        help=(
            "Maximum rate for solving the latest slider target "
            f"(default: {DEFAULT_IK_UPDATE_HZ:g} Hz)."
        ),
    )
    parser.add_argument(
        "--render-hz",
        type=float,
        default=DEFAULT_RENDER_HZ,
        help=(
            "MeshCat interpolation refresh rate "
            f"(default: {DEFAULT_RENDER_HZ:g} Hz)."
        ),
    )
    parser.add_argument(
        "--smooth-time",
        type=float,
        default=DEFAULT_SMOOTH_TIME,
        help=(
            "Seconds used to smoothly retarget each IK result; 0 disables "
            f"smoothing (default: {DEFAULT_SMOOTH_TIME:g})."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = _arguments()
    description_root = args.description_root.expanduser().resolve()
    urdf_path = _resolve_urdf(args.urdf, description_root)
    model = fr3.RobotModel(str(urdf_path), args.end_effector)
    print(f"IK posture_gain (null-space): {args.posture_gain:g}")
    print(
        "Qt smoothing: "
        f"IK {args.ik_update_hz:g} Hz, render {args.render_hz:g} Hz, "
        f"smooth_time {args.smooth_time:g} s"
    )

    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("7-DoF Robot Control Sim")

    visualizer = None
    if not args.no_meshcat:
        from fr3_control_sim.visualizer import Visualizer

        visualizer = Visualizer(
            model,
            urdf_path,
            description_root,
            open_browser=not args.no_open_browser,
        )
        print(f"MeshCat: {visualizer.url}")

    window = Fr3SimWindow(
        model,
        visualizer,
        urdf_path,
        initial_mode=args.mode,
        posture_gain=args.posture_gain,
        ik_update_hz=args.ik_update_hz,
        render_hz=args.render_hz,
        smooth_time=args.smooth_time,
    )
    window.show()
    return app.exec()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        app = QApplication.instance()
        if app is not None:
            QMessageBox.critical(None, "Robot simulation error", str(exc))
        raise
