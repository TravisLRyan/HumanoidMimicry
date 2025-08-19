from setuptools import find_packages, setup
from glob import glob

package_name = 'pose_tracker'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install URDF and meshes
        ('share/' + package_name + '/urdf', glob('../../urdf/*.urdf*')),
        ('share/' + package_name + '/urdf/meshes', glob('../../urdf/meshes/*.STL')),

        ('share/' + package_name + '/assets/g1', glob('../../assets/g1/*.urdf*')),
        ('share/' + package_name + '/assets/g1/meshes', glob('../../assets/g1/meshes/*.STL')),

        #launch files
        ('share/' + package_name + '/launch', glob('../../launch/*.py')),

        #rviz configuration
        ('share/' + package_name + '/rviz', glob('../../rviz/*.rviz')),
	],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_tracker_node = pose_tracker.pose_tracker:main',
            'pose_follower_node = pose_tracker.pose_follower:main',
        ],
    },
)
