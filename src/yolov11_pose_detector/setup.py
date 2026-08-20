from setuptools import find_packages, setup

package_name = 'yolov11_pose_detector'

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
    maintainer='radxa',
    maintainer_email='radxa@todo.todo',
    description='ROS 2 package for YOLOv11 Pose Detection and visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_detector_node = yolov11_pose_detector.pose_detector_node:main',
            'opencv_cam_node = yolov11_pose_detector.opencv_cam_node:main'
        ],
    },
)
