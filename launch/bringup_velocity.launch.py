import os
import yaml

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.events import matches_action

import pathlib
from launch import LaunchIntrospector
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

import lifecycle_msgs.msg

def generate_launch_description():

    package_name_ = "reference_model_ros2"
    config_name_ = "params_vel_model.yaml"

    config_dir = os.path.join(get_package_share_directory(package_name_), 'config')
    param_config_velocity = os.path.join(config_dir,config_name_)
    with open(param_config_velocity, 'r') as f:
        params = yaml.safe_load(f)[package_name_]


    reference_model = LifecycleNode(
        package=package_name_,              # must match name in config -> YAML
        executable='ref_model_velocity',
        name='reference_model_velocity',    # must match node name in config -> YAML
        output='screen',
        namespace='',
        parameters=[params]
    )

    # Create the launch configuration variables
    # Make the node take the 'configure' transition
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(reference_model),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the node take the 'activate' transition
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=reference_model, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(reference_model),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([
        reference_model,
        configure_event,
        activate_event
    ])