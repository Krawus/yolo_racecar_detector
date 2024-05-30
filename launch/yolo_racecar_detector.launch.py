# Copyright 2024 akrawczyk
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

def launch_setup(context, *args, **kwargs):
    param_path = LaunchConfiguration('yolo_racecar_detector_param_file').perform(context)
    if not param_path:
        param_path = PathJoinSubstitution(
            [FindPackageShare('yolo_racecar_detector'), 'config', 'yolo_racecar_detector.param.yaml']
        ).perform(context)


    # pass correct - absolute path to model and engine
    params = {}
    with open(param_path, 'r') as file:
        params = yaml.safe_load(file)
        params['/**']['ros__parameters']['onnx_model_path'] = PathJoinSubstitution(
            [FindPackageShare('yolo_racecar_detector'),
            params['/**']['ros__parameters']['onnx_model_path']]
        ).perform(context)

        params['/**']['ros__parameters']['trt_engine_path'] = PathJoinSubstitution(
            [FindPackageShare('yolo_racecar_detector'),
            params['/**']['ros__parameters']['trt_engine_path']]
        ).perform(context)
    
    yolo_racecar_detector_node = Node(
        package='yolo_racecar_detector',
        executable='yolo_racecar_detector_node_exe',
        name='yolo_racecar_detector_node',
        parameters=[
            params['/**']['ros__parameters']
        ],
        output='screen',
        remappings=[
            ('/in/image', '/sensing/camera/image_raw'),
            ('/out/image', '/yolo_racecar_detector/viz/image_with_bboxes'),
            ('/out/objects', '/yolo_racecar_detector/objects')],
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
    )

    return [
        yolo_racecar_detector_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('yolo_racecar_detector_param_file', '')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
