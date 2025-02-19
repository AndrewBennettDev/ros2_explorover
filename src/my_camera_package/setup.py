from setuptools import find_packages, setup

package_name = 'my_camera_package'

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
    maintainer='andrewbennett',
    maintainer_email='andrewbennettdev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = my_camera_package.video_publisher:main',
            'video_subscriber = my_camera_package.video_subscriber:main',
        ],
    },
)
