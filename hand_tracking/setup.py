from setuptools import setup

package_name = 'hand_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pes2ug22cs193',
    maintainer_email='itsmedrrg@gmail.com',
    description='Hand detection and tracking using ROS2, OpenCV, RViz',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_detection_node = hand_tracking.hand_detection_node:main',
            'hand_tracking_node = hand_tracking.hand_tracking_node:main',
        ],
    },
)

