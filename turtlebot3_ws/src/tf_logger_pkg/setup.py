from setuptools import setup
import os
from glob import glob

package_name = 'tf_logger_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='TF logger node that writes x,y,yaw,time to ~/tf_log.csv',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_logger_node = tf_logger_pkg.tf_logger:main'
        ],
    },
)
