from setuptools import find_packages, setup

package_name = 'cobot1_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tmdwodl',
    maintainer_email='tmdwodl12@gmail.com',
    description='협동로봇1 피킹 시스템 노드 그래프',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = cobot1_system.camera_node:main',
            'central_node = cobot1_system.central_node:main',
            'robot_arm_node = cobot1_system.robot_arm_node:main',
            'db_node = cobot1_system.db_node:main',
        ],
    },
)
