from setuptools import find_packages, setup
import glob

package_name = 'tcebot_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf/include', [
        'urdf/include/tcebot_materials.xacro',
         ]),
        ('share/' + package_name + '/urdf', [
        'urdf/tcebot_gz.urdf.xacro',
         ]),
        ('share/' + package_name + '/launch/include', [
        'launch/include/gz_ros_bridge.launch.py',
        'launch/include/spawn_robot.launch.py',
         ]),
        ('share/' + package_name + '/launch', [
        'launch/tcebot_gz.launch.py',
        'launch/slam_toolbox_online_async.launch.py',
        'launch/view_frames.launch.py',
         ]),
        ('share/' + package_name + '/tcebot_gz/launch_tools', [
        'tcebot_gz/launch_tools/__init__.py',
        'tcebot_gz/launch_tools/substitutions.py'
        ]),
        ('share/' + package_name + '/rviz', glob.glob('rviz/*')),
        ('share/' + package_name + '/worlds', glob.glob('worlds/*')),
        ('share/' + package_name + '/config', glob.glob('config/*')),
        ('share/' + package_name + '/config_gui', glob.glob('config_gui/*')),
        ('share/' + package_name + '/env-hooks', glob.glob('env-hooks/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhanush',
    maintainer_email='dhanushshettigar90@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
