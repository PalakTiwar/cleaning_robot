from setuptools import find_packages, setup

package_name = 'yolo_inference_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='palak',
    maintainer_email='sudhirtiwari0916@gmail.com',
    description='YOLOv8 inference node for room-related object detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_inference_node = yolo_inference_node.yolo_inference_node:main',
            'room_type_detector = room_type_detector.room_type_detector:main',
        ],
    },
)

