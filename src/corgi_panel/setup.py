import os
from glob import glob
from setuptools import setup

package_name = 'corgi_panel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexc',
    maintainer_email='alexc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This makes your Python script executable
            'corgi_control_panel = corgi_panel.corgi_control_panel:main',
        ],
    },
)