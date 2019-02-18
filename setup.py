# -*- coding: utf-8 -*-

# Learn more: https://github.com/mithunnallana/setup.py

from setuptools import setup, find_packages


with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='tscc',
    version='0.1.0',
    description='Time scaled collision cone',
    long_description=readme,
    author='Mithun Babu Nallana',
    author_email='mithun.babu@research.iiit.ac.in',
    url='https://github.com/MithunNallana/Time-Scaled-Collision-Cone',
    license=license,
    packages=find_packages(exclude=('tests', 'docs'))
)
