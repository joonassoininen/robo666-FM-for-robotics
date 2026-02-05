"""Record functions for teleoperation, policy deployment and more in a manipulator environment.

This module should be used in conjunction with the `RecordingManager` class.

This file is heavily modified by our group compared to the original crisp file.
This is done because X-VLA produces different action compared to the SmolVLA. 
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING, Callable

import numpy as np
import torch
from lerobot.configs.train import TrainPipelineConfig
from lerobot.policies.factory import get_policy_class
from lerobot.processor.pipeline import PolicyProcessorPipeline

from crisp_gym.util.control_type import ControlType
from crisp_gym.util.lerobot_features import numpy_obs_to_torch

if TYPE_CHECKING:
    from multiprocessing.connection import Connection

    from crisp_gym.manipulator_env import ManipulatorBaseEnv, ManipulatorCartesianEnv
    from crisp_gym.teleop.teleop_robot import TeleopRobot
    from crisp_gym.teleop.teleop_sensor_stream import TeleopStreamedPose


def make_teleop_streamer_fn(env: ManipulatorCartesianEnv, leader: TeleopStreamedPose) -> Callable:
    """Create a teleoperation function for the leader robot using streamed pose data."""
    prev_pose = leader.last_pose
    first_step = True

    def _fn() -> tuple:
        """Teleoperation function to be called in each step.

        This function computes the action based on the current end-effector pose
        or joint values of the leader robot, updates the gripper value, and steps
        the environment.

        Returns:
            tuple: A tuple containing the observation from the environment and the action taken.
        """
        nonlocal prev_pose, first_step
        if first_step:
            first_step = False
            prev_pose = leader.last_pose
            return None, None

        pose = leader.last_pose
        action_pose = pose - prev_pose
        prev_pose = pose

        
        try:
            gripper = leader.last_gripper
        except RuntimeError:
            gripper = 0.0 

        action = np.concatenate(
            [
                list(action_pose.position) + list(action_pose.orientation.as_euler("xyz")),
                [gripper],
            ]
        )
        obs, *_ = env.step(action, block=False)
        return obs, action

    return _fn


def make_teleop_fn(env: ManipulatorBaseEnv, leader: TeleopRobot) -> Callable:
    """Create a teleoperation function for the leader robot.

    This function returns a Callable that can be used to control the leader robot
    in a teleoperation manner. It computes the action based on the difference
    between the current and previous end-effector pose or joint values, and
    updates the gripper value based on the leader gripper's value.

    Args:
        env (ManipulatorBaseEnv): The environment in which the leader robot operates.
        leader (TeleopRobot): The teleoperation leader robot instance.

    Returns:
        Callable: A function that, when called, performs a step in the environment
        and returns the observation and action taken.
    """
    prev_pose = leader.robot.end_effector_pose
    prev_joint = leader.robot.joint_values
    first_step = True

    def _fn() -> tuple:
        """Teleoperation function to be called in each step.

        This function computes the action based on the current end-effector pose
        or joint values of the leader robot, updates the gripper value, and steps
        the environment.

        Returns:
            tuple: A tuple containing the observation from the environment and the action taken.
        """
        nonlocal prev_pose, prev_joint, first_step
        if first_step:
            first_step = False
            prev_pose = leader.robot.end_effector_pose
            prev_joint = leader.robot.joint_values
            return None, None

        pose = leader.robot.end_effector_pose
        joint = leader.robot.joint_values
        action_pose = pose - prev_pose
        action_joint = joint - prev_joint
        prev_pose = pose
        prev_joint = joint

        gripper = leader.gripper.value if leader.gripper is not None else 0.0

        action = None
        if env.ctrl_type is ControlType.CARTESIAN:
            action = np.concatenate(
                [
                    list(action_pose.position) + list(action_pose.orientation.as_euler("xyz")),
                    [gripper],
                ]
            )
        elif env.ctrl_type is ControlType.JOINT:
            action = np.concatenate(
                [
                    action_joint,
                    [gripper],
                ]
            )
        else:
            raise ValueError(
                f"Unsupported control type: {env.ctrl_type}. "
                "Supported types are 'cartesian' and 'joint' for delta actions."
            )

        obs, *_ = env.step(action, block=False)
        return obs, action

    return _fn

def inference_worker(
    conn: Connection,
    pretrained_path: str,
    env: ManipulatorBaseEnv,
):  # noqa: ANN001
    """Policy inference process: loads policy on GPU, receives observations via conn, returns actions, and exits on None.

    Args:
        conn (Connection): The connection to the parent process for sending and receiving data.
        pretrained_path (str): Path to the pretrained policy model.
        dataset_metadata (LeRobotDatasetMetadata): Metadata for the dataset, if needed.
        env (ManipulatorBaseEnv): The environment in which the policy will be applied.
    """
    
    # inference_worker function is built based on the original crisp_gym record_functions.py
    # inference_worker function, but there are numerous changes to make the X-VLA deploying working
    import logging
    from pathlib import Path

    import torch
    from lerobot.configs.train import TrainPipelineConfig
    from lerobot.policies.factory import get_policy_class
    from lerobot.processor.pipeline import PolicyProcessorPipeline

    from crisp_gym.util.lerobot_features import numpy_obs_to_torch
    
    _FIXED_DOMAIN_ID: int | None = None
    
    _LAST_ACTION = None

    def _zero_action(env) -> torch.Tensor:
    # create a (1, action_dim) tensor of zeros
        try:
            a = env.action_space.sample()
            a = np.zeros_like(np.asarray(a), dtype=np.float32)
        except Exception:
        
            a = np.zeros((7,), dtype=np.float32)
        return torch.from_numpy(a).unsqueeze(0)
    
    def pack_state_for_xvla(batch: dict) -> dict:
       
        #XVLA in some versions expects a single `observation.state` vector.
        #We build it from the separate state fields to match the 72-token checkpoint.
        
        if "observation.state" in batch:
            return batch

        keys = [
            "observation.state.cartesian",
            "observation.state.gripper",
            "observation.state.joint",
            "observation.state.target",
        ]
        if all(k in batch and torch.is_tensor(batch[k]) for k in keys):
            batch["observation.state"] = torch.cat([batch[k] for k in keys], dim=-1)
        return batch

    
    def domain_probe(policy, batch: dict, max_id: int = 16) -> int | None:
    
        #Try domain_id values until policy.select_action stops throwing.
        
        if "domain_id" not in batch or not torch.is_tensor(batch["domain_id"]):
            logging.info("[DomainProbe] No domain_id tensor in batch.")
            return None

        for did in range(max_id):
            test = dict(batch)
            test["domain_id"] = batch["domain_id"].clone()
            test["domain_id"].fill_(did)
            try:
                _ = policy.select_action(test)
                logging.info("[DomainProbe] SUCCESS domain_id=%d", did)
                return did
            except RuntimeError as e:
                logging.info(
                    "[DomainProbe] FAIL domain_id=%d: %s",
                    did,
                    str(e).splitlines()[-1],
                )
        return None
    
    def move_to_device(batch: Any, device: torch.device) -> Any:
        
        #Move tensors to device, keep non-tensors (str/int/float/etc.) unchanged.
        #Supports nested dict/list/tuple structures.
        
        if torch.is_tensor(batch):
            return batch.to(device)
        if isinstance(batch, dict):
            return {k: move_to_device(v, device) for k, v in batch.items()}
        if isinstance(batch, list):
            return [move_to_device(v, device) for v in batch]
        if isinstance(batch, tuple):
            return tuple(move_to_device(v, device) for v in batch)
        return batch
    
        
    def log_batch_shapes(batch: dict) -> None:
        for k, v in batch.items():
            if torch.is_tensor(v):
                logging.info("[Batch] %s: %s %s", k, v.dtype, tuple(v.shape))
            else:
                logging.info("[Batch] %s: %s", k, type(v).__name__)


    repo_id = pretrained_path 
    REV = "322fdb0ae6b58ecfb644e447136113b8810c3e68"  
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    pretrained_path_obj = Path(pretrained_path)

    #  Policy
    train_config = TrainPipelineConfig.from_pretrained(repo_id, revision=REV)
    policy_cls = get_policy_class(train_config.policy.type)
    policy = policy_cls.from_pretrained(repo_id, revision=REV)
    policy.to(device).eval()
    policy.reset()

    # Processor pipeline (load from policy_preprocessor.json using LeRobot API)
    try:
        preprocessor = PolicyProcessorPipeline.from_pretrained(
            repo_id,
            revision=REV,
            config_filename="policy_preprocessor.json",
        )
    except TypeError:
        # Some versions use a different kwarg name or default filename.
        preprocessor = PolicyProcessorPipeline.from_pretrained(
            repo_id,
            revision=REV,
        )

    if hasattr(preprocessor, "to") and callable(getattr(preprocessor, "to")):
        preprocessor.to(device)
    elif hasattr(preprocessor, "set_device") and callable(getattr(preprocessor, "set_device")):
        preprocessor.set_device(device)
    elif hasattr(preprocessor, "device"):
        try:
            preprocessor.device = device
        except Exception:
            pass

    logging.info(
        "[Inference] Loaded processor pipeline (%s).",
        type(preprocessor).__name__,
    )

    # Warm-up
    #warmup_obs_raw = ensure_obs_state(env.observation_space.sample())
    warmup_obs_raw = env.observation_space.sample()
    with torch.inference_mode():
        obs_torch = numpy_obs_to_torch(warmup_obs_raw)
        obs_torch = move_to_device(obs_torch, device)
        obs_preprocessed = preprocessor(obs_torch)
        log_batch_shapes(obs_preprocessed)
        
        ae = getattr(getattr(policy.model, "transformer", None), "action_encoder", None)
        if ae is None:
            logging.info("[Model] No transformer.action_encoder found")
        else:
            w = getattr(ae, "weight", None)
            if torch.is_tensor(w):
                 logging.info("[Model] action_encoder.weight.shape: %s", tuple(w.shape))
            else:
                 logging.info("[Model] action_encoder.weight type: %s", type(w).__name__)
            if hasattr(ae, "input_size"):
                 logging.info("[Model] action_encoder.input_size attr: %s", ae.input_size)
        
        obs_preprocessed = pack_state_for_xvla(obs_preprocessed)
        _ = policy.select_action(obs_preprocessed)

    logging.info("[Inference] Warm-up complete.")

    while True:
    	# Program gets a new observation from the model
        obs_raw = conn.recv()
        if obs_raw is None:
            break
        if not isinstance(obs_raw, dict):
            logging.warning("[Inference] Got non-dict from pipe: %r (%s)", obs_raw, type(obs_raw).__name__)

            
            if obs_raw == "reset":
                policy.reset()	# Resets the policy
                _LAST_ACTION = _zero_action(env)  # Creates action with just zeros
                conn.send(_LAST_ACTION)
                continue

           
            if _LAST_ACTION is None:
                _LAST_ACTION = _zero_action(env)
            conn.send(_LAST_ACTION)
            continue
        
        with torch.inference_mode():
            obs_torch = numpy_obs_to_torch(obs_raw)
            obs_torch = move_to_device(obs_torch, device)
            obs_preprocessed = preprocessor(obs_torch)
            
            obs_preprocessed = pack_state_for_xvla(obs_preprocessed)
            # Model picks the correct action based on the observations:
            action = policy.select_action(obs_preprocessed)
            
            _LAST_ACTION = action.detach().to("cpu", dtype=torch.float32).contiguous()

        conn.send(_LAST_ACTION)

    conn.close()
    logging.info("[Inference] Worker shutting down.")


def make_policy_fn(env: ManipulatorBaseEnv, parent_conn: Connection) -> Callable:
    """Create a function to apply a policy in the environment using multiprocessing.

    This function returns a Callable that, when invoked, observes the current state
    of the environment, sends the observation to the inference worker via pipe,
    receives the action, and steps the environment with that action.

    Args:
        env (ManipulatorBaseEnv): The environment in which the policy will be applied.
        parent_conn (Connection): The connection to the inference worker for sending observations

    Returns:
        Callable: A function that, when called, performs a step in the environment
        using the policy and returns the observation and action taken.
    """

    def _fn() -> tuple:
        """Function to apply the policy in the environment.

        This function observes the current state of the environment, sends the observation
        to the inference worker, receives the action, and steps the environment.

        Returns:
            tuple: A tuple containing the observation from the environment and the action taken.
        """
        # We changed make_policy_fn -> _fn function completely to make the X-VLA deploying work
        # With our simulation. Changes are done, because original function expects straight robot
        # Action from the model, but the X-VLA returned only nonscaled deltas of the end effector
        # cartesian pose. 
        
        # This mismatch might be cause of the outdated crisp_gym version, but this function works 
        # With X-VLA
        
        
        obs_raw = env.get_obs()

        # Send observation to inference worker and receive action
        parent_conn.send(obs_raw)
        #action = parent_conn.recv().squeeze(0).to("cpu").numpy()

        action = parent_conn.recv()

        nonlocal_prev = getattr(_fn, "_prev_action7", None)

        a = np.asarray(action, dtype=np.float32).reshape(-1)

        # take first 7 (assume [cart6, grip1])
        if a.shape[0] < 7:
            logging.warning("Policy action too small (%d). Using zeros.", a.shape[0])
            a7 = np.zeros((7,), dtype=np.float32)
        else:
            a7 = a[:7].copy()

        
        a7[:6] = np.tanh(a7[:6])              # cart/rot
        a7[6] = np.tanh(a7[6])                # gripper raw

        # Scales the model output value manually, which should be done automatically, but for some
        # reason it didn't work
        scale = np.array([0.006, 0.006, 0.003, 0.01, 0.01, 0.01, 1.0], dtype=np.float32)
        a7[:6] *= scale[:6]

        # Remaps the model output gripper value to work with our environment 
        a7[6] = (a7[6] + 1.0) * 0.5  # [-1,1] -> [0,1]

        # low-pass filter to prevent jerks
        if nonlocal_prev is None:
            filtered = a7
        else:
            alpha = 0.2
            filtered = nonlocal_prev.copy()
            filtered[:6] = (1 - alpha) * nonlocal_prev[:6] + alpha * a7[:6]
            filtered[6] = a7[6]
        
        
        # ------ For action printing: --------------
        pos = filtered[:3]
        rot = filtered[3:6]
        grip = filtered[6]

        grip_status = "OPEN  " if grip > 0.3 else "CLOSED"

        print(f"\r[ACTION] XYZ: {pos[0]:+6.4f} {pos[1]:+6.4f} {pos[2]:+6.4f} | "
              f"RPY: {rot[0]:+5.3f} {rot[1]:+5.3f} {rot[2]:+5.3f} | "
              f"GRIP:{grip:4.2f} ({grip_status})", end="", flush=True)      
    

        _fn._prev_action7 = filtered

        # clip to action space (if possible)
        try:
            filtered = np.clip(filtered, env.action_space.low, env.action_space.high)
        except Exception:
            pass

        action_to_step = filtered

        try:
            env.step(action_to_step, block=False)
        except Exception as e:
            logging.exception(f"Error during environment step: {e}")

        return obs_raw, action_to_step

    return _fn
