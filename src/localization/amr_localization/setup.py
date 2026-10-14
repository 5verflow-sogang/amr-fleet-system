from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_localization'

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
    maintainer='민지',
    maintainer_email='TODO@example.com',
    description='AMCL/EKF 설정, Kidnapped Robot 복구, GT 대비 RMSE 측정',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'eval_node = amr_localization.eval_node:main',
            'eval_offline = amr_localization.eval_offline:main',
            'kidnap_recovery = amr_localization.kidnap_recovery_node:main',
            'kidnap_trial = amr_localization.kidnap_trial_node:main',
        ],
    },
)
