"""Provides configuration classes for different robot types."""

from dataclasses import dataclass, field

import numpy as np
class RobotConfig:
    joint_names: list
    home_config: list

    base_frame: str = "base_link"
    target_frame: str = "end_effector_link"

    default_controller: str = "cartesian_impedance_controller"
    cartesian_impedance_controller_name: str = "cartesian_impedance_controller"
    joint_trajectory_controller_name: str = "joint_impedance_controller"

    target_pose_topic: str = "target_pose"
    target_joint_topic: str = "target_joint"
    current_pose_topic: str = "current_pose"
    current_joint_topic: str = "joint_states"
    current_twist_topic: str = "current_twist"

    publish_frequency: float = 50.0
    time_to_home: float = 5.0

    max_pose_delay: float = 1.0
    max_joint_delay: float = 1.0

    use_tf_pose: bool = False
    tf_retrieve_rate: float = 50.0

    use_prefix: bool = False


@dataclass
class MyRobotConfig(RobotConfig):
    joint_names: list = field(default_factory=lambda: [
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    ])
    home_config: list = field(default_factory=lambda: [
        0, -0.785, 0, -2.356, 0, 1.571, 0.785
    ])
    base_frame: str = "base"
    target_frame: str = "panda_hand"
    # target_frame: str = "panda_link7"
    target_pose_topic: str = "target_pose"
    target_joint_topic: str = "target_joint"
    current_pose_topic: str = "/current_pose"
    current_joint_topic: str = "/joint_states"
    publish_frequency: float = 50.0
    time_to_home: float = 5.0