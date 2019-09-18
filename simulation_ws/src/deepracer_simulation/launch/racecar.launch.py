# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    world_name = os.environ.get('WORLD_NAME', 'easy_track')
    world_file_name = 'racecar_' + str(world_name) + '.world'

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    deepracer_pkg_share_dir = get_package_share_directory('deepracer_simulation')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_client = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
    )
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )
    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(deepracer_pkg_share_dir, 'worlds', world_file_name), ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Set "true" to increase messages written to terminal.'),
        DeclareLaunchArgument('gdb', default_value='false',
                              description='Set "true" to run gzserver with gdb'),
        DeclareLaunchArgument('state', default_value='true',
                              description='Set "false" not to load "libgazebo_ros_state.so"'),
        gazebo_server,
        gazebo_client
    ])

if __name__ == '__main__':
    generate_launch_description()
