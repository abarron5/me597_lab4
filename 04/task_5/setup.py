from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'task_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        #(os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*'))),
        ('share/' + package_name + '/resource', ['resource/lab3_video.avi']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='me597',
    maintainer_email='barron49@purdue.edu',
    description='Image pub_sub',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_publisher = task_5.image_publisher:main',
            'object_detector = task_5.object_detector:main',
        ],
    },
)
