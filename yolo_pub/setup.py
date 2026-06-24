from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yolo_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # models 폴더 안의 .pt 파일을 install/share/yolo_pub/models 로 복사
        (os.path.join('share', package_name, 'models'),
            glob(os.path.join('models', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cam = yolo_pub.camera_publisher:main',
            'yolo = yolo_pub.yolo_detector:main',
            'sub = yolo_pub.result_subscriber:main',
            'yolo_seg = yolo_pub.yolo_seg:main',
        ],
    },
)
