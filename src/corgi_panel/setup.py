# ~/corgi_ws/corgi_ros_ws/src/corgi_panel/setup.py
import os
from glob import glob
from setuptools import setup

package_name = 'corgi_panel'

# ----------------------------------------------------------------------
# 1. Create the ament resource marker (removes the warning)
# ----------------------------------------------------------------------
resource_dir = os.path.join('share', 'ament_index', 'resource_index', 'packages')
os.makedirs(resource_dir, exist_ok=True)
open(os.path.join(resource_dir, package_name), 'a').close()

# ----------------------------------------------------------------------
# 2. Build the data_files list
# ----------------------------------------------------------------------
data_files = [
    # ament index marker
    (resource_dir, [os.path.join(resource_dir, package_name)]),

    # package.xml (required for the second warning)
    (os.path.join('share', package_name), ['package.xml']),

    # launch files
    (os.path.join('share', package_name, 'launch'),
     glob(os.path.join('launch', '*.launch*'))),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=data_files,

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexc',
    maintainer_email='alexc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'corgi_control_panel = corgi_panel.corgi_control_panel:main',
        ],
    },
)