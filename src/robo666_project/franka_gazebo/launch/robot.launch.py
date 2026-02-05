from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():

    franka_gazebo_share = FindPackageShare("franka_gazebo")
    world_path = PathJoinSubstitution([franka_gazebo_share, "config", "ft_world.sdf"])
    
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="franka_description",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="panda_arm.urdf.xacro"
        ), 
        DeclareLaunchArgument(
            "controller_config_path",
            default_value=[
                PathJoinSubstitution([FindPackageShare("franka_gazebo"), "config", "franka_controllers.yaml"])
            ],
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controller_config_path = LaunchConfiguration("controller_config_path")
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf",  description_file]),
        " arm_id:=panda hand:=true use_fake_hardware:=false sim_ignition:=true ",
    ])

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time":True}, robot_description], 
        output="screen"
    )

    #controller_manager_node = Node(
    #    package="controller_manager",
    #    executable="ros2_control_node",
    #    parameters=[robot_description, controller_config_path],
    #    output="screen",
    #)

    joint_state_broadcaster_spawner = TimerAction(
    period=2.0,  
    actions=[
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
            )
        ]
    )
    
    panda_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_position_controller", "--controller-manager", "/controller_manager"],
    )
    controller_nodes = [

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["cartesian_impedance_controller", "--inactive"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_impedance_controller", "--inactive"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gravity_compensation", "--inactive"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["pose_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["external_torques_broadcaster"],
            output="screen",
        ),
    ]
    

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "panda_arm",
            "-allow_renaming", "true"
        ],
        output="screen"
    )

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [TextSubstitution(text="-r -v 1 "), world_path]}.items(),
        )
    
    camera_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        arguments=[
            # Table camera
            "/table_camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/table_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",

            # Hand camera
            "/hand_camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/hand_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
        ],
        remappings=[
            # Table camera
            ("/table_camera", "/table_camera/image_raw"),
            ("/table_camera/camera_info", "/table_camera/camera_info"),

            # Hand camera
            ("/hand_camera", "/hand_camera/image_raw"),
        #    ("/hand_camera/camera_info", "/hand_camera/camera_info"),
        ],
        output="screen",
    )
    
    table_camera_republish_node = Node(
        package="image_transport",
        executable="republish",
        name="table_image_republisher",
        arguments=[
            "raw", "compressed",
            "--ros-args",
            "--remap", "in:=/table_camera/image_raw",
            "--remap", "out/compressed:=/table_camera/image_raw/compressed",
        ],
        output="screen",
        )

    hand_camera_republish_node = Node(
        package="image_transport",
        executable="republish",
        name="hand_image_republisher",
        arguments=[
            "raw", "compressed",
            "--ros-args",
            "--remap", "in:=/hand_camera/image_raw",
            "--remap", "out/compressed:=/hand_camera/image_raw/compressed",
        ],
        output="screen",
        )

    gripper_controller_spawner = TimerAction(
        period=4.0,  # after JS broadcaster + entity spawn
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "gripper_effort_controller",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        #controller_manager_node,
        gz_spawn_entity,
        gz_launch_description,
        #panda_position_controller_spawner,
        joint_state_broadcaster_spawner,
        gripper_controller_spawner,
        camera_bridge_node,
        table_camera_republish_node,
        hand_camera_republish_node,
    ] + controller_nodes

    return LaunchDescription(declared_arguments + nodes_to_start)

