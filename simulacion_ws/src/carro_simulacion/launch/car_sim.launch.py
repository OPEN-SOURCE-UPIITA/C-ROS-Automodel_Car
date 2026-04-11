import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Rutas de archivos
    pkg_sim = get_package_share_directory('carro_simulacion')
    world_path = os.path.join(pkg_sim, 'worlds', 'pista.world')
    urdf_path = os.path.join(pkg_sim, 'urdf', 'carro.urdf')

    # 2. Variable de Entorno para mallas (meshes)
    workspace_share = os.path.dirname(pkg_sim)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=workspace_share
    )

    # Leer el archivo URDF para el robot_state_publisher
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. Incluir Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 4. Bridge de Comunicación Único (Consolidado)
    # Conecta Sensores (Gazebo -> ROS) y Comandos de Velocidad (ROS -> Gazebo)
    main_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/carro/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/carro/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', # ROS a GZ
            '/model/carro/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )

    # 5. Nodo Robot State Publisher
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

    # 6. Nodo para Spawnear el carro en Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_path,
            '-name', 'carro',
            '-x', '7.0',   # Posición en el eje X
            '-y', '6.3',   # Posición en el eje Y
            '-z', '0.5',  # Altura
            '-Y', '3.14'   # Rotación en radianes
        ],
        output='screen'
    )

    # 7. Sim Adapter
    # Ejecuta el script de Python que traduce MotorCommand a Twist
    sim_adapter_node = Node(
        package='carro_simulacion',
        executable='sim_adapter',
        name='sim_adapter',
        output='screen'
    )

    # 8. Encoder Adapter
    # Ejecuta el script de Python que traduce odometry a encoder_data
    encoder_adapter_node = Node(
        package='carro_simulacion',
        executable='encoder_adapter',
        name='encoder_adapter_node',
        output='screen'
    )
    
    # 8. Filtro de Camara
    # Nodo para filtrar la cámara y cambiar el nombre del tópico al real
    camera_filter_node = Node(
        package='carro_simulacion',
        executable='camera_filter',
        name='camera_filter_node',
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        main_bridge,
        robot_state_publisher_node,
        spawn_entity_node,
        sim_adapter_node,
        encoder_adapter_node,
        camera_filter_node,
    ])