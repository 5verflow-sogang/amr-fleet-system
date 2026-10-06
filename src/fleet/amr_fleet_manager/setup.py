from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_fleet_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/scenarios', glob('config/scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='하연',
    maintainer_email='TODO@example.com',
    description='중앙집중 Fleet 서버: 작업 할당, Traffic Management, 교착 해소',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'fleet_manager = amr_fleet_manager.fleet_manager_node:main',
            'fake_robot = amr_fleet_manager.fake_robot_node:main',
            'fleet_metrics = amr_fleet_manager.fleet_metrics_cli:main',
            'task_generator = amr_fleet_manager.task_generator_node:main',
        ],
    },
)
