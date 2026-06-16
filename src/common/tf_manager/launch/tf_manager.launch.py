import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('tf_manager')
    yaml_path = os.path.join(pkg_dir, 'config', 'static_tf.yaml')

    return LaunchDescription([
        Node(
            package='tf_manager',
            executable='tf_manager_node',
            name='tf_manager_node',
            output='screen',
            parameters=[{
                'static_tf_yaml':    yaml_path,
                'odom_topic':        '/odom',
                'joint_topic':       '/joint_states',
                'odom_timeout_sec':  1.0,
                'joint_timeout_sec': 2.0,
                'tf_max_delay_sec':  0.1,
            }]
        )
    ])