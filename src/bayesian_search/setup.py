from setuptools import find_packages, setup

package_name = 'bayesian_search'

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
    description='Bayesian search algorithm for multi-target underwater inspection',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bayesian_search_server = bayesian_search.bayesian_search_server:main',
        ],
    },
)
