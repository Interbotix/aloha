from glob import glob
import os

from setuptools import (
    find_packages,
    setup,
)

package_name = 'aloha'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude='test'),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch/*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author_email='tonyzhao@stanford.edu',
    author='Tony Zhao',
    maintainer='Tony Zhao',
    maintainer_email='tonyzhao@stanford.edu',
    description='ALOHA: A Low-cost Open-source Hardware System for Bimanual Teleoperation',
    license='BSD',
    tests_require=['pytest'],
    # entry_points={
    #     'console_scripts': [
    #         # 'armtag_tuner_gui = aloha.armtag_tuner_gui:main',
    #         # 'picture_snapper = aloha.picture_snapper:main',
    #         # 'pointcloud_tuner_gui = aloha.pointcloud_tuner_gui:main',
    #     ],
    # },
)
