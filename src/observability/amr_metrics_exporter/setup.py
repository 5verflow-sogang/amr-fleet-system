from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_metrics_exporter'

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
    maintainer='지수',
    maintainer_email='TODO@example.com',
    description='Prometheus 지표 exporter와 교착·충돌 전후 rosbag 스냅샷',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # 'node_name = amr_metrics_exporter.node_module:main',
        ],
    },
)
