from glob import glob
import os

from setuptools import setup


package_name = 'nav2_fleet'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pierre',
    maintainer_email='adresse@gmail.com',
    description='Simulation with a fleet of robots in a warehouse environment',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'multi_picking = nav2_simple_commander.multi_picking:main',
        ],
    },
)
