from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='공동',
    maintainer_email='TODO@example.com',
    description='전체 시스템 launch 조립과 다중 로봇 네임스페이스 스폰',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # 'node_name = amr_bringup.node_module:main',
        ],
    },
)
