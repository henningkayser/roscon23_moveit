import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "use_external_cable": "true",
    }

    moveit_config = (
        MoveItConfigsBuilder("roscon23")
        .robot_description(
            file_path="description/gen3_7dof.xacro", mappings=launch_arguments
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .to_moveit_configs()
    )
    # MoveItCpp demo executable
    moveit_node = Node(
        name="moveit_examples",
        package="roscon23_moveit",
        executable="roscon23_examples",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit2_tutorials")
        + "/launch/moveit_cpp_tutorial.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_trajectory_controller",
        "robotiq_gripper_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            rviz_node,
            moveit_node,
            ros2_control_node,
        ]
        + load_controllers
    )
