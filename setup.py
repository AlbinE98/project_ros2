from setuptools import setup

package_name = 'uwb_ro_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/uwb_ro_bridge.launch.py',
            'launch/room_mapping_bringup.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/slam.yaml',
        ]),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='albin',
    maintainer_email='albin@example.com',
    description='Read-only UWB bridge: reads DWM1001 POS/DIST CSV and publishes ROS 2 topics.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'uwb_read_only_node = uwb_ro_bridge.reader:main',
            'uwb_read_only_node_launch = uwb_ro_bridge.runner:main',
            'uwb_odom_broadcaster = uwb_ro_bridge.uwb_odom_broadcaster:main',
        ],
    },
)

