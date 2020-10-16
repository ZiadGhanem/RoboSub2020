from launch import LaunchDescription
from launch_ros.actions import Node

FRAME_SHAPE = [640, 360]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robosub2020_assignment2",
            node_executable="video_streamer.py",
            node_name="video_streamer",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"dataset_path": "/home/ziadyasser/ros2_ws/src/robosub2020_assignment2/dataset/gate"},
                {"frame_shape": FRAME_SHAPE}
            ]
        ),

        Node(
            package="robosub2020_assignment2",
            node_executable="gate_detector.py",
            node_name="gate_detector",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"orange_min": [10, 50, 0]},
                {"orange_max": [35, 255, 255]},
                {"area_threshold": 300}
            ]
        ),

        Node(
            package="robosub2020_assignment2",
            node_executable="motion_planner",
            node_name="motion_planner",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_shape": FRAME_SHAPE}
            ]
        )
    ])
