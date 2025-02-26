from setuptools import find_packages, setup
import glob

package_name = 'tcebot_gz'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
        ('share/' + package_name + '/maps', glob.glob('maps/*')),
        ('share/' + package_name + '/params', glob.glob('params/*')),
        ('share/' + package_name + '/rviz', glob.glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dhanush M',
    maintainer_email='dhanushshettigar90@gmail.com',
    description='ROS 2 package for simulating the TCEBot robot in Gazebo, including navigation and SLAM support.',
    license='Apache-2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add executable scripts here if needed
        ],
    },
)
