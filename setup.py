from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['aloha'],
    package_dir={'': 'src'},
)

setup(
    long_description=open('README.md').read(),
    **setup_args
)
