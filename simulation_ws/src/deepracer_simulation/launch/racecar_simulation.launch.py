import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_name = os.environ.get('WORLD_NAME')
    world_file_name = 'racecar_' + str(world_name) + '.world'
    world = os.path.join(get_package_share_directory('deepracer_simulation'), 'worlds', world_file_name)

    return LaunchDescription([       
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so','libgazebo_ros_state.so'],
            output='screen'),
    ])
