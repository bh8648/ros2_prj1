"""두산 드라이버 + cobot1_system 노드 일괄 실행 (실로봇 테스트용).

dsr_bringup2(실로봇 연결) → 잠시 대기 → camera/db/robot_arm/central 노드 실행.

  ros2 launch cobot1_system full_system.launch.py
  ros2 launch cobot1_system full_system.launch.py host:=192.168.1.100 port:=12345 camera_index:=0

전제: dsr_bringup2 패키지가 있는 워크스페이스(~/ros2_ws)와 ws_edu를 모두 source 해야
FindPackageShare('dsr_bringup2')가 잡힌다.
  source ~/ros2_ws/install/setup.bash
  source ~/ws_cobot_pjt/ws_edu/install/setup.bash
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    model = LaunchConfiguration('model')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    camera_index = LaunchConfiguration('camera_index')
    model_path = LaunchConfiguration('model_path')
    confidence_threshold = LaunchConfiguration('confidence_threshold')

    decls = [
        DeclareLaunchArgument('mode', default_value='real'),
        DeclareLaunchArgument('model', default_value='m0609'),
        DeclareLaunchArgument('host', default_value='192.168.1.100'),
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('camera_index', default_value='2',
                              description='cv2.VideoCapture 카메라 인덱스'),
        DeclareLaunchArgument('model_path', default_value='trained_yolo26_seg_best.pt'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.75'),
    ]

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dsr_bringup2'), 'launch', 'dsr_bringup2_rviz.launch.py'])),
        launch_arguments={
            'mode': mode, 'model': model, 'host': host, 'port': port,
        }.items(),
    )

    app_nodes = [
        Node(package='cobot1_system', executable='camera_node', name='camera_node',
             output='screen', parameters=[{
                 'camera_index': camera_index,
                 'model_path': model_path,
                 'confidence_threshold': confidence_threshold,
             }]),
        Node(package='cobot1_system', executable='db_node', name='db_node',
             output='screen'),
        Node(package='cobot1_system', executable='robot_arm_node', name='robot_arm_node',
             output='screen'),
        Node(package='cobot1_system', executable='central_node', name='central_node',
             output='screen'),
    ]

    # robot_arm_node의 connect()가 컨트롤러보다 먼저 돌면 응답을 기다리며 멈추므로
    # 드라이버가 뜰 시간을 준 뒤 앱 노드를 실행한다.
    delayed_app = TimerAction(period=8.0, actions=app_nodes)

    return LaunchDescription(decls + [driver, delayed_app])
