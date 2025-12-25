from setuptools import find_packages
from setuptools import setup

setup(
    name='moveit_msgs',
    version='2.7.1',
    packages=find_packages(
        include=('moveit_msgs', 'moveit_msgs.*')),
)
