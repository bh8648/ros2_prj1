from setuptools import find_packages, setup

package_name = 'cobot1'

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
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_basic = cobot1.move:main',
            'move_periodic = cobot1.move_periodic:main',
            'block_moving = cobot1.block1_1:main',
            'force_test = cobot1.force_test:main',
            'grip_test = cobot1.grip_test:main',
        ],
    },
)
