from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_security'

model_files = []
for root, dirs, files in os.walk('models'):
    for file in files:
        path = os.path.join(root, file)
        install_dir = os.path.join(
            'share',
            package_name,
            root
        )
        model_files.append((install_dir, [path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'world'),
            glob('world/*')
        ),
    ] + model_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='alitalha.105897@gmail.com',
    description='Autonomous security and reconnaissance system',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'explorer = autonomous_security.explorer:main',
            'yolo_detector = autonomous_security.yolo_detector:main',
        ],
    },
)
