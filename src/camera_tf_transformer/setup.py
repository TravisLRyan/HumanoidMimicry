from setuptools import setup

package_name = 'camera_tf_transformer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='your-email@example.com',
    description='Transforms PointCloud2 to pelvis frame',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_transformer = camera_tf_transformer.pointcloud_transformer:main',
        ],
    },
)
