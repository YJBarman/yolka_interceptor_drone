from setuptools import find_packages, setup

package_name = 'camera_viewer'

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
    maintainer='joy',
    maintainer_email='joy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
         'viewer = camera_viewer.viewer:main',
        'controller = camera_viewer.controller:main',
        'vision_follower = camera_viewer.vision_follower:main',
        'yolo_viewer = camera_viewer.yolo_viewer:main',
         'offboard_takeoff = camera_viewer.offboard_takeoff:main',
          'vision_px4_follower = camera_viewer.vision_px4_follower:main',
        ],
    },
)
