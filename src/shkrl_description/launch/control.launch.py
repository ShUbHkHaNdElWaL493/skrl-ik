from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context):

    robot_ip = LaunchConfiguration("robot_ip")
    joint_controller = LaunchConfiguration("joint_controller")
    rviz_config = LaunchConfiguration("rviz_config")
    ur_series = EnvironmentVariable('UR_SERIES').perform(context)

    nodes = []

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("shkrl_description"), "launch", "rsp.launch.py"])),
        launch_arguments = {"robot_ip" : robot_ip}.items()
    )
    nodes.append(robot_state_publisher_node)

    control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters = [
            PathJoinSubstitution([FindPackageShare("shkrl_description"), "config", f"{ur_series}_controllers.yaml"])
        ],
        remappings = [(
            "~/robot_description", "/robot_description"
        )],
        output = "screen"
    )
    nodes.append(control_node)

    dashboard_client_node = Node(
        package = "ur_robot_driver",
        executable = "dashboard_client",
        name = "dashboard_client",
        output = "screen",
        emulate_tty = True,
        parameters = [{
            "robot_ip": robot_ip,
            "receive_timeout": 20.0
        }]
    )
    nodes.append(dashboard_client_node)

    robot_state_helper_node = Node(
        package = "ur_robot_driver",
        executable = "robot_state_helper",
        name = "ur_robot_state_helper",
        output = "screen",
        parameters = [{
            "headless_mode": True,
            "robot_ip": robot_ip
        }],
    )
    nodes.append(robot_state_helper_node)

    urscript_interface_node = Node(
        package = "ur_robot_driver",
        executable = "urscript_interface",
        parameters = [{"robot_ip": robot_ip}],
        output = "screen",
    )
    nodes.append(urscript_interface_node)

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
        "ur_configuration_controller",
    ]

    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "forward_effort_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tool_contact_controller",
    ]

    controllers_active.append(joint_controller.perform(context))
    controllers_inactive.remove(joint_controller.perform(context))

    active_controllers_spawner_node = Node(
        package = "controller_manager",
        executable = "spawner",
        name = "active_controllers_spawner",
        arguments = [
            "--controller-manager",
            "/controller_manager"
        ]
        + controllers_active
    )
    nodes.append(active_controllers_spawner_node)

    inactive_controllers_spawner_node = Node(
        package = "controller_manager",
        executable = "spawner",
        name = "inactive_controllers_spawner",
        arguments = [
            "--controller-manager",
            "/controller_manager",
            "--inactive"
        ]
        + controllers_inactive,
    )
    nodes.append(inactive_controllers_spawner_node)
    
    frequency = 1.0
    if ur_series == "cb3":
        frequency = 125.0
    elif ur_series == "e-series":
        frequency = 500.0
    commander_node = Node(
        package = "shkrl_description",
        executable = "commander",
        output = "screen",
        parameters = [{
            "frequency" : frequency,
            "joint_controller" : joint_controller.perform(context)
        }]
    )
    nodes.append(commander_node)

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "log",
        arguments = ["-d", rviz_config],
    )
    nodes.append(rviz_node)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value = "192.168.56.101",
            description = "IP address by which the robot can be reached."
        ),
        DeclareLaunchArgument(
            "joint_controller",
            default_value = "scaled_joint_trajectory_controller",
            description = "Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value = PathJoinSubstitution([FindPackageShare("shkrl_description"), "rviz", "view.rviz"]),
            description = "Rviz config file (absolute path) to use when launching rviz."
        ),
        OpaqueFunction(function=launch_setup)
    ])