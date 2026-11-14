from setuptools import find_packages, setup

package_name = 'imu_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='delivery',
    maintainer_email='vignesh.modigunta@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_publisher = imu_package.imu_node:main',
            'imu_covariance_estimator = imu_package.imu_covariance_estimator:main',
            'calibrate_magnetometer = imu_package.calibrate_magnetometer:main',
        ],
    },
)
