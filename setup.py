from setuptools import find_packages, setup

package_name = 'adam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oluwaniyi',
    maintainer_email='oluwaniyi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adam_control_node = adam.adam_control_node:main',
            'voice_turtlesim = adam.voice_turtlesim:main',
            'voice_control1 = adam.voice_control1:main',
            'robot_controller1 = adam.robot_controller1:main',
            'self_voice_control = adam.self_voice_control:main',
            'self_voice_scanner = adam.self_voice_scanner:main',
            'rviz_voice_control = adam.rviz_voice_control:main',
            'turtlesim_control = adam.turtlesim_control:main',
            'adam_direction_distance = adam.adam_direction_distance:main',
            'self_voice_scanner_main = adam.self_voice_scanner:main',
            'adam_voice_control = adam.adam_voice_control:main',
            'adam_closest_distance = adam.adam_closest_distance:main',
        ],
    },
)
