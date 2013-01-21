#!/usr/bin/env python
from distutils.core import setup

setup(
    version = '0.0.1',
    scripts = ['src/hauler_vision_controller.py'],
    packages = ['hauler_vision'],
    package_dir={'':'src'}
)
