from setuptools import find_packages, setup

package_name = 'experiment'

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
            "navigation_test = experiment.navigation_test:main",
            "localization_test = experiment.localization_test:main",
            "heading_cmd_vel = experiment.heading_cmd_vel:main",
            "dummy_tb3_server = experiment.dummy_tb3_action_server:main",
            "dummy_tb3_client = experiment.dummy_tb3_action_client:main",
            "dummy_tb3_control = experiment.dummy_tb3_control:main",
        ],
    },
)
