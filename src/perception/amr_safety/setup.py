from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_safety'

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
    description='긴급정지, Safety Zone, E-Stop, 센서 고장 감지',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'safety_node = amr_safety.safety_node:main',
            'estop_button = amr_safety.estop_button_node:main',
        ],
    },
)
