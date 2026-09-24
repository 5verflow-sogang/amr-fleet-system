from setuptools import find_packages, setup

package_name = 'amr_perception_core'

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
    maintainer='연우',
    maintainer_email='TODO@example.com',
    description='Pinhole 3D 변환, KF/AKF, TTC/Probabilistic TTC, VO/ORCA (ROS 비의존 라이브러리)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # 'node_name = amr_perception_core.node_module:main',
        ],
    },
)
