from setuptools import find_packages, setup

package_name = 'dynamixel_controller_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test_gui.launch.py']),
    ],
    install_requires=['setuptools', 'tkinter'],
    zip_safe=True,
    maintainer='imanoob',
    maintainer_email='ymrs1122@gmail.com',
    description='GUI application for testing Dynamixel motors via dynamixel_controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_gui = dynamixel_controller_gui.dynamixel_gui:main',
        ],
    },
)
