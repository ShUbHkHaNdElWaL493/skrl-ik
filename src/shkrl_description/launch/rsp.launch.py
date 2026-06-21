from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_ip = LaunchConfiguration("robot_ip")

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name = "xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("shkrl_description"), "models", "robot.urdf.xacro"]),
        " ",
        "ur_type:=",
        PythonExpression(["'", EnvironmentVariable("UR_MODEL"), "'.lower()"]),
        " ",
        "robot_ip:=",
        robot_ip
    ])

    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "both",
        parameters = [{
            "use_sim_time" : False,
            "robot_description" : robot_description
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value = "0.0.0.0",
            description = "IP address by which the robot can be reached."
        ),
        robot_state_publisher_node
    ])