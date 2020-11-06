# -*- coding: utf-8 -*-
"""Setup file for Common Robot Interface.
"""

from setuptools import setup

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name="cri",
    version="0.1.0",
    description="Common Robot Interface",
    license="GPLv3",
    long_description=long_description,
    author="John Lloyd",
    author_email="jlloyd237@gmail.com",
    url="https://github.com/jlloyd237/cri/",
    packages=["cri", "cri.abb", "cri.ur", "cri.ur.rtde"],
	package_data={'cri.ur': ['rtde_config.xml']},
    install_requires=["numpy", "transforms3d"]
)
