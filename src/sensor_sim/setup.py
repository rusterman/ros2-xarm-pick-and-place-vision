from setuptools import find_packages, setup

package_name = 'sensor_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'temperature_publisher = sensor_sim.temperature_publisher:main',
            'alert_subscriber = sensor_sim.alert_subscriber:main',
        ],
    },
)
