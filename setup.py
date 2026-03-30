from setuptools import find_packages, setup

package_name = 'day24_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'camera_publisher = day24_perception.camera_publisher:main',
        'navigation_node = day24_perception.navigation_node:main',
        'tf2_broadcaster = day24_perception.tf2_broadcaster:main',
        'vo_node = day24_perception.vo_node:main',
        'yolo_node = day24_perception.yolo_node:main',
    ],
},
)
