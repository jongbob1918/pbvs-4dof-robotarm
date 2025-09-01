# launch/view_robot_gazebo.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro # xacro 모듈 import

def generate_launch_description():
    
    # 패키지 경로 설정
    pkg_robotarm_share = get_package_share_directory('robotarm')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # GZ_SIM_RESOURCE_PATH 환경 변수 설정
    install_dir = os.path.join(pkg_robotarm_share, '..')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + install_dir
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = install_dir

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # --- XACRO를 사용하여 로봇 모델 설명(robot_description) 생성 ---
    xacro_file = os.path.join(pkg_robotarm_share, 'urdf', 'robotarm.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()
    # -----------------------------------------------------------

    # Robot State Publisher 실행
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )

    # Gazebo에 로봇 모델 스폰
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot',
                   '-allow_renaming', 'true'],
        output='screen'
    )
    
    # 컨트롤러 스포너 실행
    spawn_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "joint_trajectory_controller"],
        output="screen",
    )

    # Gazebo와 ROS 2 토픽 브릿지
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # 카메라 토픽과 TF(odom->base_link) 토픽을 브릿징합니다.
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/model/my_robot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        remappings=[('/model/my_robot/tf', '/tf')], # Gazebo의 TF를 ROS의 /tf로 리매핑
        output='screen'
    )
    
    # --- RViz2 실행 노드 추가 ---
    rviz2_config_file = os.path.join(pkg_robotarm_share, 'rviz', 'view_robot.rviz') # rviz 설정 파일 경로 (파일을 만들어야 함)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz2_config_file], # rviz 설정 파일이 있다면 이 줄의 주석을 해제
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    # -----------------------------

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        spawn_controllers,
        bridge,
        rviz2_node, # RViz2 노드를 launch 설명에 추가
    ])