from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['task_manager'],
    package_dir={'': 'task_manager'},
    requires=['rospy']
)

setup(**setup_args)
