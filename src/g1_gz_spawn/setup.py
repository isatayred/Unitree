from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'g1_gz_spawn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/g1_gz_spawn']),
        ('share/g1_gz_spawn', ['package.xml']),
        (os.path.join('share', 'g1_gz_spawn', 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', 'g1_gz_spawn', 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', 'g1_gz_spawn', 'scripts'),
            glob('scripts/*.py')),
        (os.path.join('share', 'g1_gz_spawn', 'models'),
            glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)
