import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from crisp_py.gripper.gripper import Gripper, GripperConfig


JOINT_STATES_TOPIC = "/joint_states"
GRIPPER_COMMAND_TOPIC = "/gripper_position_controller/commands"
FINGER1 = "panda_finger_joint1"
FINGER2 = "panda_finger_joint2"
MIN_VAL = 0.0
MAX_VAL = 0.04
READY_TIMEOUT = 10.0


class OneShotIndex(Node):
    def __init__(self, joint_name: str):
        super().__init__("resolve_index_" + joint_name)
        self.joint_name = joint_name
        self.index: Optional[int] = None
        self._done = False
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._cb, 10)

    def _cb(self, msg: JointState):
        if not msg.name:
            return
        try:
            self.index = msg.name.index(self.joint_name)
            self._done = True
        except ValueError:
            self.get_logger().warn(f"{self.joint_name} not in /joint_states; names: {list(msg.name)}")
            self._done = True  # fail fast

    def resolve(self, timeout_sec: float = 5.0) -> int:
        t0 = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now().nanoseconds / 1e9) - t0 > timeout_sec:
                raise TimeoutError(f"Timeout waiting for /joint_states to contain {self.joint_name}")
        if self.index is None:
            raise RuntimeError(f"Could not find {self.joint_name} in /joint_states")
        return self.index


def main():
    rclpy.init()
    try:
        # Resolve index for finger1 (for feedback only)
        res1 = OneShotIndex(FINGER1)
        idx1 = res1.resolve(timeout_sec=5.0)
        res1.destroy_node()

        # CRISP gripper tracks finger1 normalized [0..1] → raw [MIN_VAL..MAX_VAL]
        cfg = GripperConfig(
            min_value=MIN_VAL,
            max_value=MAX_VAL,
            command_topic=GRIPPER_COMMAND_TOPIC,   # we will also publish to this directly
            joint_state_topic=JOINT_STATES_TOPIC,
            index=idx1,
        )
        gripper = Gripper(gripper_config=cfg)
        gripper.wait_until_ready(timeout=READY_TIMEOUT)

        # Direct command publisher (2 joints)
        node = gripper.node  # reuse same node
        pub = node.create_publisher(Float64MultiArray, GRIPPER_COMMAND_TOPIC, 10)

        def send_both(norm_target: float):
            """Publish two-joint command; keep equal magnitudes."""
            gripper.set_target(norm_target)  # updates internal target/smoothing
            # compute current smoothed raw command from gripper internals
            # next publish tick will send 1 value; we override by sending 2 values here
            raw = cfg.min_value + (cfg.max_value - cfg.min_value) * float(norm_target)
            msg = Float64MultiArray()
            msg.data = [raw, raw]  # same magnitude for both fingers
            pub.publish(msg)

        print(f"Gripper ready. Current (norm): {gripper.value:.3f}")

        print("Open...")
        for _ in range(10):
            send_both(1.0)
            time.sleep(0.1)

        print("Close...")
        for _ in range(10):
            send_both(0.0)
            time.sleep(0.1)

        print("Toggle 3x...")
        for _ in range(3):
            for _ in range(10):
                send_both(1.0); time.sleep(0.1)
            for _ in range(10):
                send_both(0.0); time.sleep(0.1)

        print("Done.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
