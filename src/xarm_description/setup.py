from setuptools import setup
from glob import glob

package_name = 'xarm_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/xarm_description']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/meshes/xarm6/visual', glob('meshes/xarm6/visual/*')),
        ('share/' + package_name + '/meshes/xarm6/collision', glob('meshes/xarm6/collision/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rustam',
    maintainer_email='ywspeakdb@gmail.com',
    description='xArm6 URDF, mesh files, and visualization launch',
    license='BSD',
    entry_points={
        'console_scripts': [
            'joint_state_pub = xarm_description.joint_state_pub:main',
            'arm_demo = xarm_description.arm_demo:main',
        ],
    },
)
