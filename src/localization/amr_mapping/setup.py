from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='지수',
    maintainer_email='TODO@example.com',
    description='slam_toolbox 설정, 지도 저장(map.pgm/map.yaml), 맵 품질 평가',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'map_quality = amr_mapping.map_quality:main',
            'gt_map = amr_mapping.gt_map:main',
            'map_route_driver = amr_mapping.route_driver:main',
        ],
    },
)
