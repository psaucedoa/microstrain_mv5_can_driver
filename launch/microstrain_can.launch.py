import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import yaml

def generate_launch_description():

    #### Socketcan Receiver CAN0 Config ####

    _CAN_PARAMS_FILE = os.path.join(
      get_package_share_directory('microstrain_mv5_can_driver'),
      'config',
      'socketcan_params.yaml'
    )

    with open(_CAN_PARAMS_FILE, 'r') as file:
      can_params = yaml.safe_load(file)


    socket_can_receiver_can_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver_can0',
        namespace=TextSubstitution(text=''),
        parameters=[
            can_params
        ],
        output='screen')

    socket_can_receiver_can_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_receiver_can_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_can_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socket_can_receiver_can0_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_receiver_can_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_can_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    #### Socketcan Sender Config ####
    socket_can_sender_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socket_can_sender',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': 'can0',
            'enable_can_fd': False,
            'timeout_sec': 0.5
        }],
        output='screen')

    socket_can_sender_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_sender_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socket_can_sender_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_sender_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    ### kuebler right ###

    _MICROSTRAIN_MV5_CAN_PARAMS_FILE = os.path.join(
      get_package_share_directory('microstrain_mv5_can_driver'),
      'config',
      'microstrain_mv5_can_params.yaml'
    )

    microstrain_mv5_can_dbc_path = get_package_share_directory('microstrain_mv5_can_driver') + \
    '/launch/MV5.dbc'

    with open(_MICROSTRAIN_MV5_CAN_PARAMS_FILE, 'r') as file:
      microstrain_mv5_can_params = yaml.safe_load(file)

    microstrain_mv5_can_node = LifecycleNode(
        package='microstrain_mv5_can_driver',
        executable='microstrain_mv5_can_driver_exe',
        name='generic_imu',
        namespace=TextSubstitution(text=''),
        parameters=[
           {'dbw_dbc_file': microstrain_mv5_can_dbc_path},   
           microstrain_mv5_can_params
           ],
        output='screen',
        # arguments=["--ros-args", "--log-level", "debug"]
        )

    microstrain_mv5_can_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=microstrain_mv5_can_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(microstrain_mv5_can_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    microstrain_mv5_can_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=microstrain_mv5_can_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(microstrain_mv5_can_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )  


    return LaunchDescription([
        DeclareLaunchArgument('auto_configure', default_value='true'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        socket_can_sender_node,
        socket_can_sender_configure_event_handler,
        socket_can_sender_activate_event_handler,
        socket_can_receiver_can_node,
        socket_can_receiver_can_configure_event_handler,
        socket_can_receiver_can0_activate_event_handler,
        microstrain_mv5_can_node,
        microstrain_mv5_can_configure_event_handler,
        microstrain_mv5_can_activate_event_handler,
    ])