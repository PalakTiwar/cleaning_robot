from setuptools import find_packages, setup

package_name = 'cleaning_controller_node'

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
    maintainer='palak',
    maintainer_email='sudhirtiwari0916@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'cleaning_controller_node = cleaning_controller_node.cleaning_controller_node:main',
        ],
    },
)
