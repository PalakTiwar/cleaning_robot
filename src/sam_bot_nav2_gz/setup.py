from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sam_bot_nav2_gz'
python_package_name = 'sam_bot_nav2_gz_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[python_package_name],
    package_dir={python_package_name: python_package_name},  # Tells where the Python code lives
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='palak',
    maintainer_email='palak@example.com',
    description='YOLO detection and smart capture for room-wise robot navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saver = sam_bot_nav2_gz_py.image_tools.image_saver:main',
        ],
    },
)

