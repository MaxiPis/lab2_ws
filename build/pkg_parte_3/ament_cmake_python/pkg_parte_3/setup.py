from setuptools import find_packages
from setuptools import setup

setup(
    name='pkg_parte_3',
    version='0.0.0',
    packages=find_packages(
        include=('pkg_parte_3', 'pkg_parte_3.*')),
)
