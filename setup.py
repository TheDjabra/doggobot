from glob import glob

from setuptools import find_packages, setup

package_name = 'doggobot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/web',
            glob('web/*.html') + glob('web/*.png') + glob('web/*.svg')
            + glob('web/*.webmanifest')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hektoras Djabra',
    maintainer_email='hektoras@djabra.org',
    description='Voice-commanded autonomous robocar for MAE/ECE 148 Team 4.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arbiter_node = doggobot.arbiter_node:main',
            'voice_bridge_node = doggobot.voice_bridge_node:main',
            'perception_node = doggobot.perception_node:main',
            'follow_node = doggobot.follow_node:main',
            'behavior_node = doggobot.behavior_node:main',
        ],
    },
)
