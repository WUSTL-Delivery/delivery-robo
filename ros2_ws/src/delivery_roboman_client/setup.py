from setuptools import setup

package_name = 'delivery_roboman_client'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websocket-client'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='WebSocket client for delivery robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_client = delivery_roboman_client.robot_client:main'
        ],
    },
)