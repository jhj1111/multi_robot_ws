# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os, xacro, math

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def euler_to_quaternion(roll, pitch, yaw, rad=False):
    """
    오일러 각도(roll, pitch, yaw)를 쿼터니언(qx, qy, qz, qw)으로 변환하는 함수.
    입력 단위는 deg여야 합니다.

    :param roll: X축 회전 (deg여야)
    :param pitch: Y축 회전 (deg여야)
    :param yaw: Z축 회전 (deg여야)
    :return: (qx, qy, qz, qw) 형태의 튜플 (쿼터니언)
    """
    if not rad : roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)

    # 각도를 절반으로 줄이기
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # 쿼터니언 공식 적용
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return (qx, qy, qz, qw)

def generate_launch_description():
    # Get the urdf file
    rescue_robot_pkg = get_package_share_directory('rescue_robot')
    urdf_path = os.path.join(get_package_share_path('rescue_robot'),
                             'urdf', 'my_robot.urdf.xacro')
    
    robot_description = xacro.process_file(urdf_path)

    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, -90.0)
    x_pose = LaunchConfiguration('x_pose', default='-4.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')

    x_rad = LaunchConfiguration('x_rad', default=0.0)
    y_rad = LaunchConfiguration('y_rad', default=0.0)
    z_rad = LaunchConfiguration('z_rad', default=math.radians(-90.0))

    robot_spawner_cmd = Node(
        package='rescue_robot',
        executable='spawn_robot',
        arguments=[robot_description.toxml(), 'robot01'],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'arm_robot', 'file', robot_description.toxml(),
                   '-x', x_pose, '-y', y_pose, '-z', z_pose, '-R', x_rad, '-P', y_rad, '-Y', z_rad],
        output='screen'
    )

    ld = LaunchDescription()

    # Declare the launch options
    # ld.add_action(declare_x_position_cmd)
    # ld.add_action(declare_y_position_cmd)

    # Add any conditioned actions
    #ld.add_action(robot_spawner_cmd)
    ld.add_action(spawn_entity)

    return ld