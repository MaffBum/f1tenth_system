from setuptools import setup

package_name = 'localization_comparing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/compare_localizers.launch.py']),
        ('share/' + package_name + '/launch', ['launch/amcl_logger_launch.py']),
        ('share/' + package_name + '/launch', ['launch/pf_logger_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Compares AMCL and custom Particle Filter localization with metrics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amcl_logger = localization_comparing.amcl_logger:main',
            'pf_logger = localization_comparing.pf_logger:main',
        ],
    },
)
