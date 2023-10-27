import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='robot1',
    version='0.0.0',
    packages=find_packages(
        include=('robot1', 'robot1.*')),
)
