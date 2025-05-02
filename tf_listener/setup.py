# setup.py
from setuptools import setup

package_name = 'tf_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # only setuptools here—remove rclpy and tf2_msgs
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathan',
    maintainer_email='nathanruslim@gmail.com',
    description='Logs /tf transforms into CSV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_logger = tf_listener.tf_listener:main',
        ],
    },
)

