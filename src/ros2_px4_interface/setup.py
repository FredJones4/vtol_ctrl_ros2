from setuptools import find_packages, setup

package_name = 'ros2_px4_interface'

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
    maintainer='cmhales',
    maintainer_email='cmhales@byu.edu',
    description='Grabs System State and feeds motor and servo control (plus trim) to PX4.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_px4_interface = ros2_px4_interface.ros2_px4_interface:main',
            'offboard_test = ros2_px4_interface.OffBoardTest:main',
        ],
    },
)
