from setuptools import setup

package_name = 'inverted_pendulum_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_robot_launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/inverted_pendulum.xacro', 'urdf/inverted_pendulum.urdf', 'urdf/sensors.xacro', 'urdf/material_calculations.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dockeruser',
    maintainer_email='dockeruser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotary_encoder_node = inverted_pendulum_simulation.rotary_encoder_node:main',
        ],
    },
)
