from setuptools import setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'ultralytics',
    ],
    zip_safe=True,
    maintainer='user',
    description='Perception node',
    license='TODO',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    entry_points={
        'console_scripts': [
            'perception_node = perception.perception_node:main',
            'lane_detection_node = perception.lane_detection_node:main',
            'debug_image_fusion_node = perception.debug_image_fusion_node:main',
            'ipm_view_node = perception.ipm_view_node:main',
        ],
    },
)