from setuptools import find_packages, setup

package_name = 'sensor_model'

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
    maintainer='talal',
    maintainer_email='talal.alotaibi@utexas.edu',
    description='Sensor data publishing using Open3D based ray casting',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor_publisher = sensor_model.sensor_publisher:main',
            'depth_subscriber_test = sensor_model.depth_subscriber_test:main',
        ],
    },
)
