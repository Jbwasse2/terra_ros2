from setuptools import setup

package_name = 'terra_camera'

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
    maintainer='justin',
    maintainer_email='jbwasse2@illinois.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'camera_publisher = terra_camera.camera:main',
                'video_publisher = terra_camera.video_streamer:main',
                'waypoint_publisher = terra_camera.waypoint:main',
                'dummy_waypoint_publisher = terra_camera.waypoint:dummy_main',
                'terra_comm_twist = terra_camera.terra_comm:twist_main',
        ],
    },
)
