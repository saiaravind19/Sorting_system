from setuptools import setup

package_name = 'fleet_launcher'

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
    maintainer='sai',
    maintainer_email='bplacearavind@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'del_hub_pkg_simulator = fleet_launcher.del_hub_pkg_simulator:main',
            'robot_visualizer = fleet_launcher.robot_visualizer:main'

        ],
    },
)
