from setuptools import setup
from glob import glob
import os

package_name = 'pallet_ground_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this to install model files
        (os.path.join('share', package_name, 'models'), glob(os.path.join(package_name, 'models', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alaa',
    maintainer_email='alaahatoum@outlook.com',
    description='Pallet and ground detection node using YOLOv8',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pallet_ground_node = pallet_ground_package.pallet_ground_node:main'
        ],
    },
)
