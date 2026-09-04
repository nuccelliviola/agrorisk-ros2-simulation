from setuptools import find_packages, setup

package_name = 'agro_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/agro_bringup_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Viola Nuccelli',
    maintainer_email='232223180+nuccelliviola@users.noreply.github.com',
    description=(
        'Bringup unificato della demo drone-rover AgroRisk: Webots + '
        'pipeline di missione agro_mission, avviati nell\'ordine corretto.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
