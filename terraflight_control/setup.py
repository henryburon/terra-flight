from setuptools import find_packages, setup

package_name = 'terraflight_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/base_station.launch.xml',
                                   'launch/robot.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henry Buron',
    maintainer_email='henryburon2024@u.northwestern.edu.com',
    description='Control terraflight robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_station = terraflight_control.base_station:base_station_entry',
            'robot_control = terraflight_control.robot_control:robot_control_entry'
        ],
    },
)
