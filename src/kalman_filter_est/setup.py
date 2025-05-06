from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kalman_filter_est'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            # [package_name]),
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yaml]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AndresM/DanielC',
    maintainer_email='andresfmc223@gmail.com/dancorpa@gmail.com',
    description='F112th|2025-1 mobile robotics class project',
    license='N/N',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kalman_estimator = kalman_filter_est.kalman_estimator:main',
        ],
    },
)
