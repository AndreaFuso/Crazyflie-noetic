from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['crazy_common_py', 'crazyflie_drone', 'crazyflie_manager', 'crazyflie_simulator'],
    package_dir={'': 'src'}
)
setup(**d)