from setuptools import find_packages, setup

package_name = 'target_tracker'

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
             'pose_printer = target_tracker.pose_printer:main',
             'talker = target_tracker.talker:main',
    'listener = target_tracker.listener:main',
             'position_publisher = target_tracker.position_publisher:main',
'position_subscriber = target_tracker.position_subscriber:main',
'tracker = target_tracker.tracker:main',
'follower = target_tracker.follower:main',
'moving_target = target_tracker.moving_target:main',
        ],
    },
)
