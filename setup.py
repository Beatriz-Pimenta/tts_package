from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tts_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # For the Audio Folder
        ('share/' + package_name + '/audio', []),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='portatil',
    maintainer_email='beatrizifpimenta@tecnico.ulisboa.pt',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_node = tts_package.tts_node:main',
            'tts_publisher = tts_package.publisher_node:main',
        ],
    },
)