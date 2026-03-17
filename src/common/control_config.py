from dataclasses import dataclass
from typing import Literal

import numpy as np


@dataclass
class ControlConfig:
    n_joints: int
    t_intv: float
    move_robot: bool
    filter_kind: Literal[
        "original",
        "target",
        "state_and_target_diff",
        "moveit_servo_humble",
        "control_and_target_diff",
        "feedback_pd_traj",
        "none",
        "filter_target_from_target_but_diff_from_control",
    ]
    n_windows: int
    use_interp: bool
    delay_for_interpolation: float
    use_first_speed_limit: bool
    use_second_speed_limit: bool
    eff_speed_limits: np.ndarray
    eff_accel_limits: np.ndarray
    abs_joint_soft_limit: np.ndarray
    target_state_abs_joint_diff_limit: list
    stopped_velocity_eps: float
    use_normalize_target_to_nearest: bool
    control_interface: Literal["position", "velocity"]
