from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb3_nav2_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarumt2204',
    maintainer_email='simoncornelius16@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb3_nav2_commander = tb3_nav2_commander.tb3_nav2_commander:main',
            'testing_service = tb3_nav2_commander.testing_service:main',
            'tb3_nav2_followpath = tb3_nav2_commander.tb3_nav2_followpath:main',
        ],
    },
)
