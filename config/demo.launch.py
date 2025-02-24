from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("simple_arm")
        .robot_description(file_path="config/simple_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/simple_arm.srdf")
        .trajectory_execution(file_path="config/simple_arm_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", moveit_config.rviz_config_path],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    moveit_config.kinematics,
                ],
            ),
            Node(
                package="moveit_ros_planning",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    moveit_config.kinematics,
                ],
            ),
        ]
    )
