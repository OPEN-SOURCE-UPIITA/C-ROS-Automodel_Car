import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declarar el argumento de modo (Default: Modo 1)
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='1',
        description='Modo 1: Full Sensores | Modo 2: Solo Encoders'
    )
    mode = LaunchConfiguration('mode')

    # 2. Rutas de archivos
    pkg_sim = get_package_share_directory('carro_simulacion')
    world_path = os.path.join(pkg_sim, 'worlds', 'pista.world')
    urdf_path = os.path.join(pkg_sim, 'urdf', 'carro.urdf')

    # 3. Variable de Entorno para mallas (meshes)
    # Le decimos a Gazebo que busque modelos en la carpeta 'share' global del workspace
    workspace_share = os.path.dirname(pkg_sim)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=workspace_share
    )

    # Leer el archivo URDF para el robot_state_publisher
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 4. Incluir Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 5. Bridges de Comunicación (La "Tabla de Ruteo")
    base_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=IfCondition(PythonExpression([mode, ' == 1'])),
        arguments=[
            '/model/carro/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/carro/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    encoder_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=IfCondition(PythonExpression([mode, ' == 2'])),
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )

    # 6. Nodo Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True} # Sincroniza el tiempo con Gazebo
        ]
    )

    # 7. Nodo para "Spawnear" (Aparecer) el carro en Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_path,           # Usamos la ruta directa del archivo
            '-name', 'carro',             # Debe coincidir con el prefijo de tus bridges
            '-x', '0.0', '-y', '0.0', '-z', '0.5' 
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,                 # Debe cargar antes que Gazebo
        mode_arg,
        gz_sim,
        base_bridge,
        sensor_bridge,
        encoder_bridge,
        robot_state_publisher_node,
        spawn_entity_node
    ])