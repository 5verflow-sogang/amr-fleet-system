from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_fleet_decentralized'

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
    maintainer='하연, 이진',
    maintainer_email='TODO@example.com',
    description='분산 협상 기반 작업 할당과 중앙 서버 장애 시 폴백 (연구 1 비교군)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'task_auction_node = amr_fleet_decentralized.task_auction_node:main',
        ],
    },
)
