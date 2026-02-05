import numpy as np
import time
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from crisp_py.robot import Pose, Robot
from frankapandarobot.robot_config import MyRobotConfig
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

from crisp_py.gripper.gripper import Gripper, GripperConfig

JOINT_STATES_TOPIC = "/joint_states"
GRIPPER_COMMAND_TOPIC = "/gripper_effort_controller/commands"
FINGER1_NAME = "panda_finger_joint1"
MIN_VAL = 0.0
MAX_VAL = 0.04
READY_TIMEOUT = 10.0

class IndexResolver(Node):
    """Resolve index of FINGER1_NAME from /joint_states once."""

    def __init__(self, joint_name: str):
        super().__init__("resolve_" + joint_name)
        self._joint_name = joint_name
        self.index = None
        self._done = False
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._cb, 10)

    def _cb(self, msg: JointState):
        if not msg.name:
            return
        try:
            self.index = msg.name.index(self._joint_name)
        except ValueError:
            self.get_logger().warn(f"{self._joint_name} not in /joint_states; names: {list(msg.name)}")
        finally:
            self._done = True

    def resolve(self, timeout_sec: float = 5.0) -> int:
        t0 = time.time()
        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout_sec:
                raise TimeoutError(f"Timeout: /joint_states did not contain {self._joint_name}")
        if self.index is None:
            raise RuntimeError(f"Could not find {self._joint_name} in /joint_states")
        return self.index

class PS5Teleop(Node):
    def __init__(self):
        super().__init__("ps5_teleop")

        # === Initialize robot (same as sporadic_poses.py) ===
        self.my_robot_config = MyRobotConfig()
        self.robot = Robot(robot_config=self.my_robot_config, namespace="")
        self.robot.wait_until_ready()

        self.get_logger().info("Homing robot...")
        # self.robot.home()

        # Switch to Cartesian impedance controller
        #self.robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
        
        #self.robot.set_target_joint(self.robot.joint_values)

        # === Set identical controller parameters as in sporadic_poses.py ===
        #self.robot.cartesian_controller_parameters_client.set_parameters(
        #    [
        #        ("task.k_pos_x", 2000.0),
        #        ("task.k_pos_y", 2000.0),
        #        ("task.k_pos_z", 2000.0),
        #        ("task.k_rot_x", 3000.0),
        #        ("task.k_rot_y", 3000.0),
        #        ("task.k_rot_z", 3000.0),
        #        ("task.d_pos_x", -1.0),
        #        ("task.d_pos_y", -1.0),
        #        ("task.d_pos_z", -1.0),
        #        ("task.d_rot_x", -1.0),
        #        ("task.d_rot_y", -1.0),
        #        ("task.d_rot_z", -1.0),
        #       ("task.error_clip.x", 0.005),
        #        ("task.error_clip.y", 0.005),
        #        ("task.error_clip.z", 0.005),
        #        ("task.error_clip.rx", 0.006),
        #        ("task.error_clip.ry", 0.006),
        #        ("task.error_clip.rz", 0.006),
        #        ("nullspace.projector_type", "kinematic"),
        #        ("nullspace.stiffness", 450.0),
        #        ("nullspace.damping", -1.0),
        #        ("use_operational_space", True),
        #        ("use_local_jacobian", True),
        #        ("use_friction", False),
        #        ("use_gravity_compensation", False),
        #    ]
        #)

        self.get_logger().info("Controller parameters set to sporadic_poses.py defaults.")

        # === Initialize pose and control variables ===
        self.pose = self.robot.end_effector_pose.copy()
        self.home_pose = Pose(
            position=np.array([
                0.5142586090890201,
                0.002680603957996082,
                1.1658895726105398,
            ]),
            orientation=Rotation.from_quat([
                0.9950965726570878,      # x
                -0.08900067048847418,    # y
                0.00005088300100585775, # z
                0.04314729597135166,     # w
            ])
        )

        self.homing = False
        self.homing_start_pose = None
        self.homing_start_time = 0.0
        self.homing_duration = 3.0

        self.step = 0.002     # translation step per joystick unit
        self.rota = 0.004    # rotation step per joystick unit
        self.deadzone = 0.08

        # Joystick inputs
        self.dx = self.dy = self.dz = 0.0
        self.rx = self.ry = self.rz = 0.0

        # Subscribe to /joy topic
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # 20 Hz control update (same as sporadic rate)
        self.rate = 1.0 / 100.0
        self.timer = self.create_timer(self.rate, self.update_robot)

        # --- Gripper ---
        idx = self._resolve_finger_index(FINGER1_NAME)

        gcfg = GripperConfig(
            min_value=MIN_VAL,
            max_value=MAX_VAL,
            command_topic=GRIPPER_COMMAND_TOPIC,
            joint_state_topic=JOINT_STATES_TOPIC,
            index=idx,
        )

        self.opened = 1.0
        
        self.pose_pub = self.create_publisher(PoseStamped, "/teleop_pose", 10)
        self.grip_pub = self.create_publisher(Float32, "/teleop_gripper", 10)

        self.gripper_pub = self.create_publisher(Float64MultiArray, GRIPPER_COMMAND_TOPIC, 10)

        self.get_logger().info("PS5 teleop initialized. Use left stick + triggers to move.")
    

    def _send_both(self, norm_target: float):

        msg = Float64MultiArray()

        if self.opened < 0.5:     # close
            F = 7.0
            msg.data = [-F, -F]   
        else:                     # open / idle
            msg.data = [7.0, 7.0]

        self.gripper_pub.publish(msg)
        g = Float32()
        g.data = float(self.opened)
        self.grip_pub.publish(g)
    
    
    def _resolve_finger_index(self, joint_name: str) -> int:
        resolver = IndexResolver(joint_name)
        try:
            idx = resolver.resolve(timeout_sec=5.0)
            self.get_logger().info(f"{joint_name} index in /joint_states: {idx}")
            return idx
        finally:
            resolver.destroy_node()

    def go_home(self):

        self.get_logger().info("Going to home position")

        if self.homing:
            self.get_logger().info("Already homing, ignoring new request.")
            return

        try:
            self.dx = 0
            self.dy = 0
            self.dz = 0
            self.rx = 0
            self.ry = 0
            self.rz = 0
            
            self.homing_start_pose = self.pose.copy()
            self.homing_start_time = time.time()

            distance = np.linalg.norm(self.home_pose.position - self.homing_start_pose.position)

            # duration = max(2.0, min(3.0, dist / 0.05))
            self.homing = True
        
        except Exception as e: 
            self.get_logger.info("Failed to go to home position")

    
    def joy_callback(self, msg: Joy):
        axes = msg.axes

        def axis(i):
            return axes[i] if i < len(axes) else 0.0

        # Apply deadzone
        def dz(val):
            return val if abs(val) > self.deadzone else 0.0

        
        self.dx = dz(axis(1)) * self.step     # Forward/back
        self.dy = dz(axis(0)) * self.step     # Left/right
        self.dz = dz(axis(4)) * self.step     # Right stick vertical

        lt = axis(2)  # L2
        rt = axis(5)  # R2
        self.rx = dz(rt - lt) * (self.rota / 2.0)
        self.ry = dz(axis(7)) * self.rota if len(axes) > 7 else 0.0
        self.rz = dz(axis(6)) * self.rota if len(axes) > 6 else 0.0

        if len(msg.buttons) > 0 and msg.buttons[0] == 1:  # X -> close
            self.opened = 0.0
        if len(msg.buttons) > 2 and msg.buttons[2] == 1:  # Triangle -> open
            self.opened = 1.0

        # Button[1] (O on PS5) = shutdown
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            self.get_logger().info("Shutdown button pressed.")
            self.robot.home()
            self.robot.shutdown()
            rclpy.shutdown()

        if len(msg.buttons) > 3 and msg.buttons[3] == 1:
            self.go_home()

    # === Main control update ===
    def update_robot(self):
        try:
        
            now = time.time()

            if self.homing:
                t = now - self.homing_start_time

                if t >= self.homing_duration:
                    alpha = 1.0
                else:
                    alpha = t/self.homing_duration
                
                start_pos = self.homing_start_pose.position
                target_pos = self.home_pose.position
                self.pose.position = (1.0 - alpha) * start_pos + alpha * target_pos

                self.pose.orientation = self.home_pose.orientation

                
                if alpha >= 1.0:
                    self.homing = False

                    self.dx = 0
                    self.dy = 0
                    self.dz = 0
                    self.rx = 0
                    self.ry = 0
                    self.rz = 0
            else: 
            
                still = (
                    abs(self.dx) < 1e-6
                    and abs(self.dy) < 1e-6
                    and abs(self.dz) < 1e-6
                    and abs(self.rx) < 1e-6
                    and abs(self.ry) < 1e-6
                    and abs(self.rz) < 1e-6
                )

                if not still:
                                       
                    delta_pos = np.array([self.dx, self.dy, self.dz])
                    if np.linalg.norm(delta_pos) > 0:
                        self.pose.position += delta_pos

                    
                    if any(abs(v) > 0 for v in [self.rx, self.ry, self.rz]):
                        rot = Rotation.from_euler("xyz", [self.rx, self.ry, self.rz], degrees=False)
                        self.pose.orientation = self.pose.orientation * rot

                    
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "teleop"

            msg.pose.position.x = float(self.pose.position[0])
            msg.pose.position.y = float(self.pose.position[1])
            msg.pose.position.z = float(self.pose.position[2])

            q = self.pose.orientation.as_quat()  # [x, y, z, w]
            msg.pose.orientation.x = float(q[0])
            msg.pose.orientation.y = float(q[1])
            msg.pose.orientation.z = float(q[2])
            msg.pose.orientation.w = float(q[3])

            self.pose_pub.publish(msg)

            
            self._send_both(self.opened)

                    
            if not hasattr(self, "_last_log_time"):
                self._last_log_time = 0.0
            now = time.time()
            if now - self._last_log_time > 2.0:
                self.get_logger().info(
                    f"Target pos={self.pose.position}, rotvec={self.pose.orientation.as_rotvec()}"
                )
                self._last_log_time = now

        except Exception as e:
            self.get_logger().warn(f"Error updating robot target: {e}")

    # === Clean shutdown ===
    def destroy_node(self):
        self.get_logger().info("Returning home and shutting down.")
        try:
            self.robot.home()
            self.robot.shutdown()
        except Exception as e:
            self.get_logger().warn(f"Shutdown error: {e}")
        super().destroy_node()


def main():
    rclpy.init()
    node = PS5Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
