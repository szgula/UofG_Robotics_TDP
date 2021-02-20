## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['GameEngine'],
    scripts=['src'],
    package_dir={'': 'src'}
)

setup(**d)
