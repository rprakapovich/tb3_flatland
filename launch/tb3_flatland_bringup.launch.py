#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    package_dir = get_package_share_directory('tb3_flatland')
    
    world_file = os.path.join(package_dir, 'world', 'tb3_world.yaml')
    robot_model = os.path.join(package_dir, 'world', 'tb3_waffle.yaml')
    rviz_config = os.path.join(package_dir, 'rviz', 'tb3_flatland.rviz')
    
    print(f"\n{'='*60}")
    print(f"FLATLAND WAREHOUSE SIMULATION - FILE CHECK")
    print(f"{'='*60}")
    print(f"World file:  {world_file}")
    print(f"  → Exists: {os.path.exists(world_file)}")
    print(f"Robot model: {robot_model}")
    print(f"  → Exists: {os.path.exists(robot_model)}")
    print(f"RViz config: {rviz_config}")
    print(f"  → Exists: {os.path.exists(rviz_config)}")
    print(f"{'='*60}\n")
    
    ld = LaunchDescription()
    
    # ========================================
    # 1. FLATLAND SERVER
    # ========================================
    flatland_server = Node(
        package='flatland_server',
        executable='flatland_server',
        name='flatland_server',
        output='screen',
        parameters=[{
            'world_path': world_file,
            'update_rate': 100.0,
            'step_size': 0.01,
            'show_viz': True,
            'viz_pub_rate': 30.0
        }],
        remappings=[('WafflePi/scan', 'scan')],
        emulate_tty=True  
    )
    
    ld.add_action(flatland_server)
    
    # ========================================
    # 2. TF STATIC TRANSFORM (map → odom)
    # ========================================
    static_pub_map_odom = Node(
        package="tf2_ros", 
        executable="static_transform_publisher",
        name="static_tf_pub_map_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output='screen'
    )
    
    ld.add_action(static_pub_map_odom)

    # ========================================
    # 3. SPAWN WORLD
    # ========================================
    spawn_world = Node(
        package='tb3_flatland',
        executable='spawn_world',
        name='spawn_world',
        output='screen',
    )
    ld.add_action(spawn_world)

    # ========================================
    # 4. SPAWN ROBOT (with delay)
    # ========================================
    spawn_robot = Node(
        package='tb3_flatland',
        executable='spawn_robot',
        name='spawn_robot',
        output='screen',
        parameters=[{
            'robot_yaml': robot_model,
            'robot_name': 'WafflePi',
            'robot_ns': '',
            'x': -2.0,
            'y': -0.5,
            'theta': 0.0
        }],
        emulate_tty=True
    )
    
    delayed_spawn1 = TimerAction(
        period=1.0,  
        actions=[
            LogInfo(msg='[LAUNCH] Spawning Robot 1...'),
            spawn_robot
        ]
    )
    # Robot spawn with delay
    ld.add_action(delayed_spawn1)
    
    # ========================================
    # 5. RVIZ (with delay after Flatland)
    # ========================================
    start_rviz = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )
    
    # Launch RViz with delay
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[LAUNCH] Starting RViz2...'),
            start_rviz
        ]
    )
    
    ld.add_action(delayed_rviz)
    
    # ========================================
    # 6. Info messages
    # ========================================
    ld.add_action(
        TimerAction(
            period=1.0,
            actions=[
                LogInfo(msg='[LAUNCH] Flatland server starting...')
            ]
        )
    )
    
    ld.add_action(
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg='[LAUNCH] All systems should be up now!')
            ]
        )
    )
    
    return ld