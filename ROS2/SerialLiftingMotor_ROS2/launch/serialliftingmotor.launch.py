from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serialliftingmotor',
            executable='serialliftingmotor_node',
            name='serialliftingmotor',
            output='screen',
            parameters=[
                {'dev': '/dev/ttyACM0'},
                {'baud': 19200},
                {'sub_cmdvel_topic': '/serial_lifting_motor/cmd_position'},
                {'pub_position_topic': '/serial_lifting_motor/cmd_position'}
            ]
        )
    ])

 
