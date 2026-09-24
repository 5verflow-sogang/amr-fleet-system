from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_tracking'

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
    maintainer='연우',
    maintainer_email='TODO@example.com',
    description='LiDAR 동적 장애물 분류·추적, 카메라 검출 연관, 충돌 예측 노드',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # 'node_name = amr_tracking.node_module:main',
        ],
    },
)
