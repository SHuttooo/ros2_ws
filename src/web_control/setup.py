import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'web_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Fichiers Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Fichiers Web (Racine)
        # CRUCIAL : On utilise une liste en compréhension avec os.path.isfile(f)
        # Cela dit : "Prends les fichiers dans web/, mais IGNORE les dossiers (comme navig)"
        # C'est ça qui empêche l'erreur de build.
        (os.path.join('share', package_name, 'web'), [f for f in glob('web/*') if os.path.isfile(f)]),
        
        # 3. Dossier Navig
        # Ici on copie le contenu de ton dossier navig
        (os.path.join('share', package_name, 'web/navig'), glob('web/navig/*')),
        
        # 4. Dossier Gallery (Optionnel, seulement si non vide)
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