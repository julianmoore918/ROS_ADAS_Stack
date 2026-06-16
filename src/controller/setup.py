from setuptools import setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    description='Controller node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'controller_node = controller.controller_node:main',
            'stanley_node = controller.stanley_node:main',
        ],
    },
)
