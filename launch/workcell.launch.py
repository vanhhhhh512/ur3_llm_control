"""Dung toan bo workcell mo phong: Gazebo + UR3e + controllers + MoveIt + RViz.

Day la launch file chay o terminal thu nhat. No khong chua logic cua bai tap,
chi ghep cac launch chinh thuc cua Universal Robots lai voi world rieng cua
workcell (ban thao tac, ba khoi, ba vung dat).

Cac include duoc boc trong GroupAction de moi launch con giu duoc launch
argument cua rieng no; neu khong RViz va move_group se khong nhan dung tham so.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")

    world_cua_bai = PathJoinSubstitution(
        [FindPackageShare("ur3_llm_control"), "worlds", "ur3_workcell.sdf"])
    rviz_cua_bai = PathJoinSubstitution(
        [FindPackageShare("ur3_llm_control"), "rviz", "workcell.rviz"])

    # Gazebo + robot + ros2_control. RViz cua launch nay bi tat vi ta dung
    # cau hinh RViz rieng di kem move_group o duoi.
    mo_phong = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch/ur_sim_control.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": "true",
            "world_file": world_cua_bai,
            "gazebo_gui": gazebo_gui,
            "launch_rviz": "false",
            "initial_joint_controller": "joint_trajectory_controller",
            # Dung file controller rieng cua bai tap (nguong bam quy dao noi long)
            "runtime_config_package": "ur3_llm_control",
            "controllers_file": "ur3_controllers.yaml",
        }.items(),
    )

    # move_group cua MoveIt 2 cho UR, kem RViz voi cau hinh rieng cua workcell
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch/ur_moveit.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": "true",
            "use_sim_time": "true",
            "launch_rviz": launch_rviz,
            "rviz_config_file": rviz_cua_bai,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e",
                              description="ur3 hoac ur3e"),
        DeclareLaunchArgument("gazebo_gui", default_value="true",
                              description="Mo cua so Gazebo (tat khi chay kiem thu khong man hinh)"),
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Mo RViz"),
        GroupAction([mo_phong]),
        GroupAction([moveit]),
    ])
