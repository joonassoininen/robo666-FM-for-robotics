# This teleoperation could be done much simpler, but there are many 
# Additional rows and functions which we used when we tried to reduce
# End effector errorenous rotations and gripper behaviour. 

# This teleoperation doesn't actually move the robot, but it send end 
# effector targets to the crisp_gym nodes, and crisp_gym uses the 
# cartesian impedance controller to actually move the robot. So this
# Teleoperation is used as leader in the training phase. 

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Float64MultiArray
from scipy.spatial.transform import Rotation
from frankapandarobot.robot_config import MyRobotConfig
from crisp_py.robot import Pose, Robot
from sensor_msgs.msg import JointState


ROBOT_POSE_TOPIC = "/current_pose"  
GRIPPER_COMMAND_TOPIC = "/gripper_effort_controller/commands"


class PS5TeleopPose(Node):
    def __init__(self):
        super().__init__("ps5_teleop_pose")

        #self.my_robot_config = MyRobotConfig()
        #self.robot = Robot(robot_config=self.my_robot_config, namespace="")
        #self.robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

        # Defines how much end effector cartesian position changes per step when controller axis are used
        self.step = 0.0015 
        # Defines how much end effector rotation changes per step when controller rotation buttons are used (defined below)
        self.rota = 0.01
        # Controller deadzone
        self.deadzone = 0.08

        # deltas
        self.dx = 0.0
        self.dy = 0.0
        self.dz_lin = 0.0

        self.rx = 0.0
        self.ry = 0.0
        self.rz = 0.0

        # Gripper opening/closing forces
        self.F_grip = 12.0
        self.F_hold = 15.0
        self.F_open = 7.0

        self.f1_name = "panda_finger_joint1"
        self.f2_name = "panda_finger_joint2"
        self.f1_idx = None
        self.f2_idx = None
        
        self.hold_width = None
        self.Kp_width = 400.0   
        self.max_extra = 20.0

        self.last_width = None
        self.width_stall_count = 0
        self.width_eps = 2e-4
        self.stall_cycles = 6

        self.min_width_for_hold = 0.002

        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)

        # integrated pose (will be initialized from robot EE pose)
        self.pos = np.zeros(3, dtype=np.float64)
        self.rot = Rotation.identity()
        self.initialized_from_robot = False

        # last robot EE pose (for initialization)
        self.robot_pos = None
        self.robot_rot = None

        # Publishers expected by TeleopStreamedPose
        self.pose_pub = self.create_publisher(PoseStamped, "/teleop_pose", 10)
        self.grip_pub = self.create_publisher(Float32, "/teleop_gripper", 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, GRIPPER_COMMAND_TOPIC, 10)

        #self.robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

        # Subscribe to joystick
        self.create_subscription(Joy, "/joy", self.joy_cb, 10)

        # Subscribe to robot EE pose
        self.create_subscription(PoseStamped, ROBOT_POSE_TOPIC, self.robot_pose_cb, 10)

        # Publish at 60 Hz
        self.create_timer(1.0 / 60.0, self.publish)
        self.grip_mode = "HOLD"
        self.roll_cmd = 0.0
        self.roll_axis_local = np.array([0.0, 0.0, 1.0])

        self.get_logger().info(
            f"Publishing /teleop_pose and /teleop_gripper; "
            f"initializing from {ROBOT_POSE_TOPIC}"
        )

    def _send_both(self):	# Publishes the gripper actions
        msg = Float64MultiArray()

        if self.grip_mode == "OPEN":
            F = float(np.clip(self.F_open, 0.0, 30.0))
            msg.data = [F, F]
        elif self.grip_mode == "GRIP":
            F = float(np.clip(self.F_grip, 0.0, 30.0))
            msg.data = [-F, -F]
        else:  # HOLD
            F = float(np.clip(self.F_hold, 0.0, 30.0))

            extra = 0.0
            if self.hold_width is not None and self.last_width is not None:
        
                err = (self.last_width - self.hold_width)  
                if err > 0.0:
                    extra = float(np.clip(self.Kp_width * err, 0.0, self.max_extra))

    
            msg.data = [-(F + extra), -(F + extra)]

        self.gripper_pub.publish(msg)

        g = Float32()
        g.data = 0.0 if self.grip_mode in ("GRIP", "HOLD") else 1.0
        self.grip_pub.publish(g)

    
    def apply_deadzone(self, v: float) -> float:    # Applies additional deadzone to controller axis
    
        if abs(v) > self.deadzone:
            return v
        else:
            return 0.0

    @staticmethod  
    def axis(axes, i: int) -> float:   # Helper function which allows only available axises to be controlled   
        if i < len(axes):
            return axes[i]
        else:
            return 0.0

    # ---------------------- callbacks ----------------------
    def robot_pose_cb(self, msg: PoseStamped):    #Store latest robot EE pose to initialize teleop origin.

        self.robot_pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )
        self.robot_rot = Rotation.from_quat(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )
    def joint_cb(self, msg: JointState):
        # Resolves the correct index of the gripper fingers
        if self.f1_idx is None:
            if self.f1_name in msg.name and self.f2_name in msg.name:
                self.f1_idx = msg.name.index(self.f1_name)
                self.f2_idx = msg.name.index(self.f2_name)
            else:
                return
        # Calculates the width of the fingers
        f1 = msg.position[self.f1_idx]
        f2 = msg.position[self.f2_idx]
        width = float(f1 + f2)

        if self.grip_mode == "GRIP":
        
            if width <= self.min_width_for_hold:
                self.grip_mode = "HOLD"
                self.hold_width = width
                self.width_stall_count = 0
            else:
                if self.last_width is not None:
                    dw = abs(width - self.last_width)
                    if dw < self.width_eps:
                        self.width_stall_count += 1
                    else:
                        self.width_stall_count = 0

                    if self.width_stall_count >= self.stall_cycles:
                        self.grip_mode = "HOLD"
                        self.hold_width = width
                        self.width_stall_count = 0

        self.last_width = width
    def joy_cb(self, msg: Joy):
        a = msg.axes
        b = msg.buttons

        # Left stick horizontal movement changes end effector x target
        self.dx = self.apply_deadzone(self.axis(a, 1)) * self.step
        # Left stick vertical movement changes end effector y target
        self.dy = self.apply_deadzone(self.axis(a, 0)) * self.step
        # Right stick vertical movement changes end effector z target
        self.dz_lin = self.apply_deadzone(self.axis(a, 4)) * self.step

        # Rotation deltas
        lt = self.axis(a, 2)   # L2
        rt = self.axis(a, 5)   # R2

        dpad = self.apply_deadzone(self.axis(a, 6))
        self.roll_cmd = dpad

        # Left arrow adds positive roll
        # Right arrow adds negative roll
        v_rt = 1.0 if dpad < 0.0 else 0.0   # left
        v_lt = 1.0 if dpad > 0.0 else 0.0   # right

        # Combine real triggers + virtual triggers
        rt_eff = self.apply_deadzone(rt) + v_rt
        lt_eff = self.apply_deadzone(lt) + v_lt

        rt_eff = float(np.clip(rt_eff, 0.0, 1.0))
        lt_eff = float(np.clip(lt_eff, 0.0, 1.0))

        self.rx = (rt_eff - lt_eff) * self.rota * 0.5

        self.ry = self.apply_deadzone(self.axis(a, 7)) * self.rota

        # Gripper
        if len(b) > 0 and b[0] == 1:   # X -> grip
            self.grip_mode = "GRIP"
        if len(b) > 2 and b[2] == 1:   # Triangle -> open
            self.grip_mode = "OPEN"

        # Optional stop
        if len(b) > 1 and b[1] == 1:   # O
            self.get_logger().info("Shutdown button pressed (teleop node only).")
            rclpy.shutdown()

    # ---------------------- main publish loop ----------------------
    def publish(self):
        # One-time initialization from robot EE pose
        if not self.initialized_from_robot:
            if self.robot_pos is not None and self.robot_rot is not None:
                self.pos = self.robot_pos.copy()
                self.rot = self.robot_rot
                self.initialized_from_robot = True
                self.get_logger().info(
                    f"Initialized teleop origin from robot EE pose on {ROBOT_POSE_TOPIC}"
                )
            else:
                pass

        # Integrate deltas
        self.pos += np.array([self.dx, self.dy, self.dz_lin], dtype=np.float64)
        
        d = self.roll_cmd * self.rota
        if abs(d) > 0.0:
            axis_world = self.rot.apply(self.roll_axis_local)      
            delta = Rotation.from_rotvec(d * axis_world)           
            self.rot = delta * self.rot 

        w_local = np.array([self.rx, self.ry, self.rz], dtype=np.float64)
        if np.linalg.norm(w_local) > 0.0:
            self.rot = self.rot * Rotation.from_rotvec(w_local)

        # Build PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "teleop" 

        msg.pose.position.x = float(self.pos[0])
        msg.pose.position.y = float(self.pos[1])
        msg.pose.position.z = float(self.pos[2])

        q = self.rot.as_quat()  # [x, y, z, w]
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])

        self.pose_pub.publish(msg)

        # sync gripper state and send gripper command + teleop_gripper
        self._send_both()


def main():
    rclpy.init()
    node = PS5TeleopPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
