from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpipe',
            executable='mpipe_webcam_node',
            name='mpipe_webcam'
        ),
        Node(
            package='mpipe',
            executable='mpipe_pose_estimator_node',
            name='mpipe_pose_estimator'
        ),
    ])