from setuptools import find_packages, setup

package_name = 'synthetic_minimal_sensors'

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
    maintainer='chris',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'synthetic_lidar        = synthetic_minimal_sensors.lidar:main',
            'synthetic_range_finder = synthetic_minimal_sensors.range_finder:main',
            'synthetic_image        = synthetic_minimal_sensors.image:main',
        ],
    },
)
