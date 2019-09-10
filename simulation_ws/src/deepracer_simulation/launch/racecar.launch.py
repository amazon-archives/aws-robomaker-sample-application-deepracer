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
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    world_name = os.environ.get('WORLD_NAME')
    world_file_name = 'racecar_' + str(world_name) + '.world'
    world = os.path.join(get_package_share_directory('deepracer_simulation'), 'worlds', world_file_name)
    return LaunchDescription([       
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so','libgazebo_ros_state.so'],
            output='screen'),
    ])

if __name__ == '__main__':
    generate_launch_description()
