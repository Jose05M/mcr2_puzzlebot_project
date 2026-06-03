import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mcr2_puzzlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ed',
    maintainer_email='eduardo.mtz1403@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_follower_controller   = mcr2_puzzlebot.line_follower_controller:main',
            'line_follower              = mcr2_puzzlebot.line_follower:main',
            'traffic_light_detector     = mcr2_puzzlebot.traffic_light_detector:main',
            'traffic_sign_detector 		= mcr2_puzzlebot.traffic_sign_detector:main',
            'image_compressor 		    = mcr2_puzzlebot.image_compressor:main',
            'puzzlebot_odometry         = mcr2_puzzlebot.puzzlebot_odometry:main',
        ],
    },
)
