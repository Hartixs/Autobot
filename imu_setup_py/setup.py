import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imu_setup_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # The following line installs all files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rbcnvamsi',
    maintainer_email='rbcnvamsi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "mpu6050_visualizer=imu_setup_py.imu_values:main",
            'eskf_node = imu_setup_py.eskf:main',
            'heading_lock_node = imu_setup_py.heading:main',
            'imu_publisher=imu_setup_py.imu_publisher:main'
        ],
    },
)