# setup.py

from setuptools import setup

package_name = 'my_robotics_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'my_robotics_project.simple_publisher',
        'my_robotics_project.simple_subscriber',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A simple ROS 2 package with publisher and subscriber nodes',
    license='License Declaration',
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robotics_project.simple_publisher:main',
            'simple_subscriber = my_robotics_project.simple_subscriber:main',
        ],
    },
)
