#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_dir = get_package_share_directory('tb3_flatland')

    world_file  = os.path.join(package_dir, 'world', 'roborace_world.yaml')
    robot_model = os.path.join(package_dir, 'world', 'umbreon.yaml')
    rviz_config = os.path.join(package_dir, 'rviz', 'umbreon_flatland.rviz')

    print(f"\n{'='*60}")
    print(f"UMBREON ROBORACE — FLATLAND SIMULATION")
    print(f"{'='*60}")
    print(f"World : {world_file}  exists={os.path.exists(world_file)}")
    print(f"Robot : {robot_model}  exists={os.path.exists(robot_model)}")
    print(f"RViz  : {rviz_config}  exists={os.path.exists(rviz_config)}")
    print(f"{'='*60}\n")

    ld = LaunchDescription()

    # 1. Flatland server
    flatland_server = Node(
        package='flatland_server',
        executable='flatland_server',
        name='flatland_server',
        output='screen',
        parameters=[{
            'world_path': world_file,
            'update_rate': 100.0,
            'step_size':   0.01,
            'show_viz':    True,
            'viz_pub_rate': 30.0,
        }],
        emulate_tty=True,
    )
    ld.add_action(flatland_server)

    # 2. Static TF  map → odom
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    ld.add_action(static_tf)

    # 3. Spawn Umbreon (after 1 s so Flatland is ready)
    spawn_robot = Node(
        package='tb3_flatland',
        executable='spawn_robot',
        name='spawn_umbreon',
        output='screen',
        parameters=[{
            'robot_yaml':  robot_model,
            'robot_name':  'Umbreon',
            'robot_ns':    'umbreon',
            'x':   1.5,
            'y':   0.5,
            'theta': 0.0,
        }],
        emulate_tty=True,
    )
    ld.add_action(TimerAction(period=1.0, actions=[
        LogInfo(msg='[LAUNCH] Spawning Umbreon...'),
        spawn_robot,
    ]))

    # 4. Umbreon controller node
    controller = Node(
        package='tb3_flatland',
        executable='umbreon_controller',
        name='umbreon_controller',
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(TimerAction(period=2.0, actions=[
        LogInfo(msg='[LAUNCH] Starting Umbreon controller...'),
        controller,
    ]))

    # 5. RViz2
    rviz = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    ld.add_action(TimerAction(period=3.0, actions=[
        LogInfo(msg='[LAUNCH] Starting RViz2...'),
        rviz,
    ]))

    return ld
