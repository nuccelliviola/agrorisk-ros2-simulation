from setuptools import find_packages, setup

package_name = 'agro_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Viola Nuccelli',
    maintainer_email='232223180+nuccelliviola@users.noreply.github.com',
    description='Nodi ROS2 per la pipeline allerta-missione-log',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agro_alert_publisher = agro_mission.agro_alert_publisher:main',
            'mission_manager = agro_mission.mission_manager:main',
            'mission_logger = agro_mission.mission_logger:main',
        ],
    },
)
