from glob import glob

from setuptools import find_packages, setup

package_name = 'car_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atlas1',
    maintainer_email='atlas1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "driver_node = car_driver.driver_node:main",
            "receiver_node = car_driver.receiver_node:main",
            "sample_controller = car_driver.sample_controller:main",
            "arm = car_driver.arm:main",
        ],
    },
)
