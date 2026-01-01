import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'web_control'

# --- FONCTION MAGIQUE ---
# Cette fonction parcourt un dossier et prépare la liste pour data_files
def package_files(data_files, directory_list):
    paths_dict = {}
    for (path, directories, filenames) in os.walk(directory_list):
        for filename in filenames:
            # On ignore les fichiers cachés du Mac (._)
            if filename.startswith('._'):
                continue
                
            file_path = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            
            if install_path in paths_dict:
                paths_dict[install_path].append(file_path)
            else:
                paths_dict[install_path] = [file_path]
                
    for key in paths_dict:
        data_files.append((key, paths_dict[key]))
    
    return data_files

# --- LISTE DE BASE ---
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

# --- AJOUT AUTOMATIQUE DU DOSSIER WEB ---
# Cela va scanner web/ et tout ajouter (css, js, images, sous-dossiers...)
# sans que tu aies besoin de modifier le setup.py à l'avenir.
package_files(data_files, 'web')

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,  # On utilise la liste générée
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
