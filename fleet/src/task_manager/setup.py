from setuptools import setup

package_name = 'task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,'task_manager/utils' ,'task_manager/interface', 'task_manager/state_machine'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'grpcio-tools',
                      'grpcio',
                      ],
    zip_safe=True,
    maintainer='Aravind',
    maintainer_email='bplacearavind@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'task_manager = task_manager.task_manager:main'
        ],
    },
)
