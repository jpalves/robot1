#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

import xacro

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('robot1'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    world = os.path.join(
        get_package_share_directory('robot1'),
        'worlds',
        'hospital.world'
    )
    
    urdf = os.path.join(get_package_share_directory('robot1'), "urdf", "verde.xacro")
    doc = xacro.parse(open(urdf))
    xacro.process_doc(doc)
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = Node(
        #condition=IfCondition(use_robot_state_pub),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=[urdf],
        remappings=remappings,
        parameters=[{"robot_description": doc.toxml(),"use_sim_time": use_sim_time}],

    )
    
    #robot_state_publisher_cmd = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
    #    ),
    #    launch_arguments={'use_sim_time': use_sim_time}.items()
    #)

    cmd_pid = Node(
        package='robot1',
        executable='cmd_pid.py',
        name='cmd_pid',
        output='screen',
    )    

    ld = LaunchDescription()

    # Add the commands to the launch description
    #ld.add_action(gzserver_cmd)
    #ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(cmd_pid)
    
    return ld
