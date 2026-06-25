"""cobot1_system 피킹 시스템 전체 노드 일괄 실행.

camera_node / db_node / robot_arm_node / central_node 를 한 번에 띄운다.

  ros2 launch cobot1_system cobot1_system.launch.py
  ros2 launch cobot1_system cobot1_system.launch.py camera_index:=0 confidence_threshold:=0.7

주의: robot_arm_node는 두산 드라이버(dsr01/m0609 컨트롤러)에 연결한다.
드라이버를 먼저 띄운 뒤 이 launch를 실행해야 한다. ui_node는 팀원 담당이라
여기 포함하지 않는다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_index = LaunchConfiguration('camera_index')
    model_path = LaunchConfiguration('model_path')
    confidence_threshold = LaunchConfiguration('confidence_threshold')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_index', default_value='2',
            description='cv2.VideoCapture 카메라 인덱스'),
        DeclareLaunchArgument(
            'model_path', default_value='trained_yolo26_seg_best.pt',
            description='YOLO 모델 파일명(또는 절대경로). 파일명만 주면 '
                        'cobot1_system/share/models 아래에서 찾는다.'),
        DeclareLaunchArgument(
            'confidence_threshold', default_value='0.75',
            description='YOLO confidence 임계값'),

        Node(
            package='cobot1_system',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_index': camera_index,
                'model_path': model_path,
                'confidence_threshold': confidence_threshold,
            }],
        ),
        Node(
            package='cobot1_system',
            executable='db_node',
            name='db_node',
            output='screen',
        ),
        Node(
            package='cobot1_system',
            executable='robot_arm_node',
            name='robot_arm_node',
            output='screen',
        ),
        Node(
            package='cobot1_system',
            executable='central_node',
            name='central_node',
            output='screen',
        ),
    ])
