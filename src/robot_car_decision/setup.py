from setuptools import setup

package_name = 'robot_car_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu@email.com',
    description='Decision package for robot car',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'decision = robot_car_decision.decision_node:main',
            'vision_sim = robot_car_decision.vision_sim:main',
            'ultrasonic_sim = robot_car_decision.ultrasonic_sim:main',
            'lidar_sim = robot_car_decision.lidar_sim:main',
        ],
    },
)
