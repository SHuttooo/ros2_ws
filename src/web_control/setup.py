import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'web_control'

setup(
    name=package_name,
    version='1.0.0',
    # --- MODIFICATION ICI : On utilise find_packages() ---
    packages=find_packages(exclude=['test']),
    # -----------------------------------------------------
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Fichiers Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Fichiers Web
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'web/gallery'), glob('web/gallery/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Interface Web ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'backend_node = web_control.backend_node:main',
        ],
    },
)
