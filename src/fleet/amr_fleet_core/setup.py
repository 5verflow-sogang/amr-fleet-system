from setuptools import find_packages, setup

package_name = 'amr_fleet_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='하연',
    maintainer_email='TODO@example.com',
    description='작업 할당(최소거리/부하균형/Hungarian), 경로 충돌 예측, 교착 탐지 (ROS 비의존 라이브러리)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # 'node_name = amr_fleet_core.node_module:main',
        ],
    },
)
