from setuptools import find_packages, setup

package_name = 'mapping_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mapper_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dhanush',
    maintainer_email='dhanushshettigar90@gmail.com',
    description='TRos2 package to demonstrate launch files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
