"""Simple example to control the gripper."""

# %%
import time
from pathlib import Path

from crisp_py.gripper.gripper import Gripper, GripperConfig

# %%

#config = GripperConfig.from_yaml("config/gripper_right.yaml")
#gripper = Gripper(namespace="right", gripper_config=config)
#config = GripperConfig.from_yaml("config/gripper_franka.yaml")
#gripper = Gripper(namespace="gripper", gripper_config=config)
#gripper.config.max_delta = 0.15
config = GripperConfig(
    min_value=0.0,
    max_value=0.04,
    command_topic="gripper/gripper_position_controller/commands",
    joint_state_topic="gripper/joint_states",
)  


gripper = Gripper(gripper_config=config)
gripper.wait_until_ready()


# %%
freq = 1.0
rate = gripper.node.create_rate(freq)
t = 0.0
#while t < 10.0:
#    print(gripper.value)
#    print(gripper.torque)
#    rate.sleep()
#    t += 1.0 / freq

# %%
gripper.value
print("Loop done", gripper.value)

gripper.close()
time.sleep(3.0)
gripper.set_target(1.0)
print("Set first target", gripper.torque)
time.sleep(3.0)
gripper.set_target(0.0)
print("Set second target", gripper.torque)
time.sleep(3.0)

for _ in range(6):
    gripper.set_target(1.0)
    time.sleep(3.0)
    gripper.set_target(0.0)
    time.sleep(3.0)
