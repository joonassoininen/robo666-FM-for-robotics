"""Script showcasing how to record data in Lerobot Format."""

import argparse
import logging

import numpy as np
import rclpy  # noqa: F401

import crisp_gym  # noqa: F401
from crisp_gym.config.home import home_close_to_table
from crisp_gym.config.path import CRISP_CONFIG_PATH
from crisp_gym.manipulator_env import ManipulatorCartesianEnv, make_env
from crisp_gym.manipulator_env_config import list_env_configs
from crisp_gym.record.record_functions import make_teleop_fn, make_teleop_streamer_fn
from crisp_gym.record.recording_manager import make_recording_manager
from crisp_gym.teleop.teleop_robot import TeleopRobot, make_leader
from crisp_gym.teleop.teleop_robot_config import list_leader_configs
from crisp_gym.teleop.teleop_sensor_stream import TeleopStreamedPose
from crisp_gym.util import prompt
from crisp_gym.util.lerobot_features import get_features
from crisp_gym.util.setup_logger import setup_logging
from crisp_gym.record.recording_manager_config import RecordingManagerConfig
from crisp_gym.util.control_type import ControlType
from crisp_py.robot import Pose
from scipy.spatial.transform import Rotation


parser = argparse.ArgumentParser(description="Record data in Lerobot Format")
parser.add_argument(
    "--repo-id",
    type=str,
    default="test",
    help="Repository ID for the dataset",
)
parser.add_argument(
    "--tasks",
    type=str,
    nargs="+",
    default=["Pick up the green cube and place it on to the red square."],
    help="List of task descriptions to record data for, e.g. 'clean red' 'clean green'",
)
parser.add_argument(
    "--robot-type",
    type=str,
    default="franka",
    help="Type of robot being used.",
)
parser.add_argument(
    "--fps",
    type=int,
    default=10,
    help="Frames per second for recording",
)
parser.add_argument(
    "--num-episodes",
    type=int,
    default=50,
    help="Number of episodes to record",
)
parser.add_argument(
    "--resume",
    action="store_true",
    default=False,
    help="Resume recording of an already existing dataset",
)
parser.add_argument(
    "--push-to-hub",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="Whether to push the dataset to the Hugging Face Hub.",
)
parser.add_argument(
    "--recording-manager-type",
    type=str,
    default="ros",
    help="Type of recording manager to use. Currently only 'keyboard' and 'ros' are supported.",
)
parser.add_argument(
    "--leader-config",
    type=str,
    default=None,
    help="Configuration name for the leader robot. Define your own configuration in `crisp_gym/teleop/teleop_robot_config.py`.",
)
parser.add_argument(
    "--follower-config",
    type=str,
    default=None,
    help="Configuration name for the follower robot. Define your own configuration in `crisp_gym/manipulator_env_config.py`.",
)
parser.add_argument(
    "--follower-namespace",
    type=str,
    default=None,
    help="Namespace for the follower robot. This is used to identify the robot in the ROS ecosystem.",
)
parser.add_argument(
    "--leader-namespace",
    type=str,
    default=None,
    help="Namespace for the leader robot. This is used to identify the robot in the ROS ecosystem.",
)
parser.add_argument(
    "--joint-control",
    action="store_true",
    help="Whether to use joint control for the robot.",
)
parser.add_argument(
    "--log-level",
    type=str,
    default="INFO",
    choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
    help="Set the logger level.",
)

parser.add_argument(
    "--use-streamed-teleop",
    action="store_true",
    help="Whether to use streamed teleop (e.g., from a phone or VR device) for the leader robot.",
)

args = parser.parse_args()

# Set up logger
logger = logging.getLogger(__name__)
setup_logging(level=args.log_level)


logger.info("Arguments:")
for arg, value in vars(args).items():
    logger.info(f"  {arg}: {value}")


# Validate arguments not passed by the user
if args.follower_namespace is None:
    args.follower_namespace = prompt.prompt(
        "Please enter the follower robot namespace (e.g., 'left', 'right', ...)",
        default="",
    )
    logger.info(f"Using follower namespace: {args.follower_namespace}")

if args.leader_namespace is None and not args.use_streamed_teleop:
    args.leader_namespace = prompt.prompt(
        "Please enter the leader robot namespace (e.g., 'left', 'right', ...)",
        default="left",
    )
    logger.info(f"Using leader namespace: {args.leader_namespace}")

if args.leader_config is None and not args.use_streamed_teleop:
    leader_configs = list_leader_configs()
    args.leader_config = prompt.prompt(
        "Please enter the leader robot configuration name.",
        options=leader_configs,
        default=leader_configs[0],
    )
    logger.info(f"Using leader configuration: {args.leader_config}")


if args.follower_config is None:
    follower_configs = list_env_configs()
    args.follower_config = prompt.prompt(
        "Please enter the follower robot configuration name.",
        options=follower_configs,
        default=follower_configs[0],
    )
    logger.info(f"Using follower configuration: {args.follower_config}")


try:
    ctrl_type = "cartesian" if not args.joint_control else "joint"

    env = make_env(
        env_type=args.follower_config,
        control_type=ctrl_type,
        namespace=args.follower_namespace,
    )
    def _sync_target_to_current_pose():
        """Make controller target = current measured EE pose to avoid jumps."""
        try:
            cur_pose = env.robot.end_effector_pose.copy()
            env.robot.set_target(pose=cur_pose)
        except Exception as e:
            logger.warning(f"Could not sync target to current pose: {e}")

    leader: TeleopRobot | TeleopStreamedPose | None = None
    if args.use_streamed_teleop:
        leader = TeleopStreamedPose()
        logger.info("Using streamed teleop for the leader robot.")
    else:
        leader = make_leader(args.leader_config, namespace=args.leader_namespace)
        leader.wait_until_ready()
        leader.config.leader.home_config = home_close_to_table
        leader.config.leader.time_to_home = 2.0
        logger.info("Using teleop robot for the leader robot. Leader is ready.")

    keys_to_ignore = []
    # keys_to_ignore += ["observation.state.joint", "observation.state.target"]
    features = get_features(env=env, ignore_keys=keys_to_ignore)
    logger.debug(f"Using the features: {features}")

    if args.use_streamed_teleop and ctrl_type != "cartesian":
        raise ValueError(
            "Streamed teleop is only compatible with Cartesian control. Please disable joint control."
        )

    #recording_manager = make_recording_manager(
    #    recording_manager_type=args.recording_manager_type,
    #    features=features,
    #    repo_id=args.repo_id,
    #    robot_type=args.robot_type,
    #    num_episodes=args.num_episodes,
    #   fps=args.fps,
    #    resume=args.resume,
    #    push_to_hub=args.push_to_hub,
    #)
    
    # Custom recording_manager configuration to get rid of the errors
    rm_cfg = RecordingManagerConfig(
        features=features,
        repo_id=args.repo_id,
        robot_type=args.robot_type,
        resume=args.resume,
        fps=args.fps,
        num_episodes=args.num_episodes,
        push_to_hub=args.push_to_hub,
        # optional overrides if you want:
        # queue_size=16,
        # writer_timeout=10.0,
        # use_sound=False,
    )

    recording_manager = make_recording_manager(
        recording_manager_type=args.recording_manager_type,
        config=rm_cfg,
    )
    recording_manager.wait_until_ready()

    logger.info("Homing both robots before starting with recording.")

    # Prepare environment and leader
    if isinstance(leader, TeleopRobot):
        leader.prepare_for_teleop()
    env.robot.config.home_config = home_close_to_table
    env.robot.config.time_to_home = 2.0
    #env.home()
    #env.reset()
    
    # Home pose definition: 
    home_pose = Pose(
        position=np.array([
            0.52,
            0.00,
            1.33,
        ]),
        orientation=Rotation.from_quat([
            0.9950840175324355,
            0.015984526112823493,
            0.09773499785544885,
            0.000403943871905746,
        ])
    )
    env.switch_controller(ControlType.CARTESIAN)
    env.initialize()
    _sync_target_to_current_pose()
    env.robot.move_to(pose=home_pose, speed=0.05)

    tasks = list(args.tasks)

    def on_start():
        """Hook function to be called when starting a new episode."""
        #env.robot.reset_targets()
        #env.reset()
        env.switch_controller(ControlType.CARTESIAN)
        logger.info(f"Follower cartesian param file: {env.config.cartesian_control_param_config}")
        env.initialize()
        _sync_target_to_current_pose()
        #env.robot.move_to(pose=home_pose, speed=0.05)

        # TODO: @danielsanjosepro: ask user for which controller to use.
        if isinstance(leader, TeleopRobot):
            try:
                leader.robot.controller_switcher_client.switch_controller(
                    "torque_feedback_controller"
                )
            except Exception:
                leader.robot.cartesian_controller_parameters_client.load_param_config(
                    CRISP_CONFIG_PATH / "control" / "gravity_compensation_on_plane.yaml"
                )
                leader.robot.controller_switcher_client.switch_controller(
                    "cartesian_impedance_controller"
                )

    def on_end():
        """Hook function to be called when stopping the recording."""
        #env.robot.reset_targets()
        #env.robot.home(blocking=False)
        #if isinstance(leader, TeleopRobot):
        #    leader.robot.reset_targets()
        #    leader.robot.home(blocking=False)
        env.robot.move_to(pose=home_pose, speed=0.05)

    with recording_manager:
        while not recording_manager.done():
            logger.info(
                f"→ Episode {recording_manager.episode_count + 1} / {recording_manager.num_episodes}"
            )

            # Create a new teleop function for each episode to reset internal variables
            teleop_fn = None
            if isinstance(leader, TeleopRobot):
                teleop_fn = make_teleop_fn(env, leader)
            elif isinstance(leader, TeleopStreamedPose) and isinstance(
                env, ManipulatorCartesianEnv
            ):
                teleop_fn = make_teleop_streamer_fn(env, leader)
            else:
                raise ValueError(
                    "Streamed teleop is only compatible with Cartesian control. Please disable joint control."
                )

            task = tasks[np.random.randint(0, len(tasks))] if tasks else "No task specified."
            logger.info(f"▷ Task: {task}")

            recording_manager.record_episode(
                data_fn=teleop_fn,
                task=task,
                on_start=on_start,
                on_end=on_end,
            )

    if isinstance(leader, TeleopRobot):
        logger.info("Homing leader.")
        leader.robot.home()
    logger.info("Homing follower.")
    #env.home()

    logger.info("Closing the environment.")
    env.close()

    logger.info("Finished recording.")

except TimeoutError as e:
    logger.exception(f"Timeout error occurred during recording: {e}.")
    logger.error(
        "Please check if the robot container is running and the namespace is correct."
        "\nYou can check the topics using `ros2 topic list` command."
    )

except Exception as e:
    logger.exception(f"An error occurred during recording: {e}.")

finally:
    if rclpy.ok():
        rclpy.shutdown()
