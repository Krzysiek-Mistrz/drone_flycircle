from setuptools import setup

package_name = 'drone_fly_circle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drone_control_xrce.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krzychu',
    maintainer_email='krzyskuar@gmail.com',
    description='flying on circle',
    license='GNU GPL V3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_fly_circle = drone_fly_circle.drone_fly_circle:main'
        ],
    },
)
