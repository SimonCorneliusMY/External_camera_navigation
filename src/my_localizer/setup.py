from setuptools import find_packages, setup

package_name = 'my_localizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'pose_aggregator = my_localizer.pose_aggregator:main',
            'static_tf = my_localizer.static_tf:main',
            'my_localizer = my_localizer.my_localizer:main',
            'my_localizer_real = my_localizer.my_localizer_real:main',
            'localizer_sim = my_localizer.localizer_sim:main',
            'localizer_real = my_localizer.localizer_real:main',
        ],
    },
)
