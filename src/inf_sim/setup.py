from setuptools import setup
import os
from glob import glob

package_name = 'inf_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yanis',
    maintainer_email='yanis@todo.todo',
    description='Package IA Detection',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference_node = inf_sim.inference_node:main',
            'object_navigator = inf_sim.object_navigator:main',
        ],
    },
)