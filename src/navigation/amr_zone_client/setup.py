from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_zone_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='이진',
    maintainer_email='TODO@example.com',
    description='로봇 측 구역 진입 처리 (ZoneRequest/ZoneGrant 클라이언트, amr_fleet_manager 서버와 짝)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'zone_client_node = amr_zone_client.zone_client_node:main',
        ],
    },
)
