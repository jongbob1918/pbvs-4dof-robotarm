import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. 경로 설정 ---
    pkg_arm_gazebo = get_package_share_directory('arm_gazebo')
    pkg_arm_description = get_package_share_directory('arm_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo가 모델을 찾을 수 있도록 리소스 경로 설정
    model_path = os.path.join(pkg_arm_gazebo, 'models')
    world_path = os.path.join(pkg_arm_gazebo, 'worlds')
    
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{model_path}:{world_path}:{gz_resource_path}'
    )

    # --- 2. Gazebo 시뮬레이션 실행 ---
    world_file = os.path.join(pkg_arm_gazebo, 'worlds', 'arm_world.sdf')

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s -v4 {world_file}'}.items()
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g'}.items()
    )

    # --- 3. 로봇 모델 준비 및 스폰 ---
    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_arm_description, 'launch', 'upload_robot.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Spawn robot
    create_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_arm',
            '-topic', 'robot_description',
        ],
        output='screen',
    )

    

    # --- 4. 브릿지 및 컨트롤러 활성화 ---
    bridge_params_file = os.path.join(pkg_arm_gazebo, 'params', 'arm_bridge.yaml')
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params_file}'],
        output='screen'
    )

    # Explicitly launch the controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(pkg_arm_gazebo, 'params', 'arm_controllers.yaml')],
        output="screen",
    )

    # Gazebo의 gz_ros2_control 플러그인이 컨트롤러를 로드하므로, 여기서는 활성화만 시켜줍니다.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # --- 5. 런치 파일 최종 구성 ---
    return LaunchDescription([
        set_gz_resource_path,
        gz_server,
        gz_client,
        upload_robot,
        create_entity,
        gz_bridge,
        controller_manager_node, # Add controller_manager_node here

    ])