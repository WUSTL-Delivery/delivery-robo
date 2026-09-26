import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hardware_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='georgengyn',
    maintainer_email='gnguyen@higusa.com',
    description='Hardware glue for running delivery-autonomy on the real robot',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'arduino_bridge = hardware_bringup.arduino_bridge:main',
            'joy_to_cmdvel = hardware_bringup.joy_to_cmdvel:main',
            'gps_datum_tf = hardware_bringup.gps_datum_tf:main',
            'bench_mppi = hardware_bringup.bench_mppi:main',
        ],
    },
)
