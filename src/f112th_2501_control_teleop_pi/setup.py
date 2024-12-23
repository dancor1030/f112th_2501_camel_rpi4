from setuptools import find_packages, setup

package_name = 'f112th_2501_control_teleop_pi'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'hardware_teleop = f112th_2501_control_teleop_pi.hardware_teleop:main'
        ],
    },
)
