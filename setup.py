from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
        packages=['peg_in_hole_demo'],
    package_dir={'': 'src'})

setup(**setup_args)
# from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup

# d = generate_distutils_setup(
#     packages=['peg_in_hole_demo'],
#     package_dir={'': 'src'}
# )
# setup(**d)
