from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'drone_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('lib/' + package_name, glob('scripts/*.py')),  
        (f'share/{package_name}/net_train/weights', glob('net_train/weights/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lacea',
    maintainer_email='lacea@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking = drone_tracking.tracking:main',
            'tracking_subscriber = drone_tracking.tracking_subscriber:main'
        ],
    },
)
