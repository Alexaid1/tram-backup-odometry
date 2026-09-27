"""Launch the tram odometry estimator; optionally play a bag and record results.

  ros2 launch tram_odometry tram_odometry.launch.py                       # live topics
  ros2 launch tram_odometry tram_odometry.launch.py bag:=/path/to/bag     # + ros2 bag play
  ros2 launch tram_odometry tram_odometry.launch.py bag:=... record:=true out_dir:=/tmp/run1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("tram_odometry")
    params = LaunchConfiguration("params_file")
    bag = LaunchConfiguration("bag")
    rate = LaunchConfiguration("rate")
    record = LaunchConfiguration("record")
    out_dir = LaunchConfiguration("out_dir")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "params.yaml")),
        DeclareLaunchArgument("use_closed_route", default_value="true",
                              description="true: pathgraph + reconstructed terminal loops; false: pathgraph only"),
        DeclareLaunchArgument("bag", default_value="", description="rosbag2 directory to play (optional)"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("record", default_value="false", description="run result_recorder"),
        DeclareLaunchArgument("out_dir", default_value="/tmp/tram_odometry_run"),
        Node(
            package="tram_odometry", executable="tram_odometry_node", name="tram_odometry_node",
            output="screen", parameters=[params, {"map.use_closed_route": LaunchConfiguration("use_closed_route")}],
        ),
        Node(
            package="tram_odometry", executable="result_recorder.py", name="result_recorder",
            output="screen", condition=IfCondition(record),
            parameters=[{"out_dir": out_dir}],
        ),
        TimerAction(period=2.0, actions=[
            ExecuteProcess(cmd=["ros2", "bag", "play", bag, "--rate", rate], output="screen",
                           condition=LaunchConfigurationNotEquals("bag", "")),
        ]),
    ])
