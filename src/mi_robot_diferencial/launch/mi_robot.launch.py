import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    pkg = get_package_share_directory('mi_robot_diferencial')
    xacro_path = os.path.join(pkg, 'urdf', 'mi_robot.urdf.xacro')
    pista = LaunchConfiguration('pista')
    modo =  LaunchConfiguration('modo') # para la etapa AEB
    twist_mux_config = os.path.join(pkg, 'config', 'twist_mux.yaml')
    es_teleop = PythonExpression(["'", modo, "' == 'teleop'"])
    es_auto = PythonExpression(["'", modo, "' == 'auto'"])
    #propria -> pista.sdf (se reemplazara por nuestra pista CAD)
    mundo_comp = os.path.join(pkg, 'worlds', 'pista_competencia.sdf')
    mundo_propio = os.path.join(pkg, 'worlds', 'pista.sdf')
    world_path = PythonExpression([ "'", mundo_comp, "' if '", pista, "' == 'competencia' else '", mundo_propio, "'"])
    # Pose inicial reglamentaria en competencia : x = -0.6, y = 0, yaw = 0
    x_spawn = PythonExpression(["'-0.60' if '", pista, "' == 'competencia' else '0.00'"])
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg, 'config', 'control.rviz')


    robot_description = Command(['xacro ', xacro_path])
    
    return LaunchDescription([
        DeclareLaunchArgument('pista', default_value='propia', choices=['propia', 'competencia'], description='Mundo a cargar'),
        DeclareLaunchArgument('modo', default_value='teleop', choices=['teleop', 'auto'], description='Modo de control del robot'),
        # PAra que Gazebo encuentre model://pista_competencia
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=os.path.join(pkg, 'models')),
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', world_path],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'mi_robot', '-x', x_spawn, '-y', '0.0', '-z', '0.01', '-Y', '0.0']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}]
        ),
        Node(
            package='mi_robot_diferencial',
            executable='comparar_odom.py',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),        
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='mi_robot_diferencial',
            executable='aeb.py',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        # Multiplexor: teleop (prioridad 100) y autónomo (10) -> /cmd_vel_mux
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_mux_config, {'use_sim_time': True}],
            remappings=[('cmd_vel_out', 'cmd_vel_mux')],
            output='screen',
        ),
        # modo:=teleop -> teclado en una ventana xterm aparte (necesita su propia terminal)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            prefix='xterm -fa Monospace -fs 12 -e',
            parameters=[{'stamped': False}],
            remappings=[('cmd_vel', 'cmd_vel_teleop')],
            condition=IfCondition(es_teleop),
        ),
        # modo:=auto -> aquí irá el seguidor de pared (siguiente etapa)
        Node(
            package='mi_robot_diferencial',
            executable='seguidor_pared.py',
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(es_auto),
        )
    ])
