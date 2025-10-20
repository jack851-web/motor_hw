# launch/demo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取URDF - 使用更可靠的方式查找xacro
    robot_description_content = Command(
        [
            "xacro",
            " ",
            PathJoinSubstitution(
                [FindPackageShare("motor_hw"), "urdf", "motor_description.urdf.xacro"]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    # controller manager参数
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("motor_hw"),
            "config",
            "control.yaml",
        ]
    )
    
    # 启动robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # 启动controller manager (移除robot_description参数)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],  # 只保留控制器参数
        output="both",
    )
    
    # 加载并启动joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # 加载并启动forward position controller
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )
    
    nodes = [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
    ]
    
    return LaunchDescription(nodes)