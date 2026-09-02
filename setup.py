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
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json')),
        ('share/' + package_name + '/web',
            glob('web/*.html') + glob('web/*.png') + glob('web/*.svg')
            + glob('web/*.webmanifest')),
    ],
    # Declared so a clone fails with a clear message rather than an ImportError
    # per node. See requirements.txt for what each group is for; the phone app
    # alone needs only fastapi and uvicorn.
    install_requires=[
        'setuptools',
        'fastapi',        # voice_bridge_node
        'uvicorn',        # voice_bridge_node
        'pyserial',       # pan_node
    ],
    extras_require={
        # Heavy, and only needed for the nodes that use them. Kept out of the
        # base install so the app can be run without pulling in depthai.
        'perception': ['depthai>=3.0.0', 'opencv-python', 'numpy'],
        'speech': ['vosk'],
        'bench': ['pyvesc'],
    },
    zip_safe=True,
    maintainer='Hektoras Djabra (Hrag Djabraian)',
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
            'stt_node = doggobot.stt_node:main',
            'safety_node = doggobot.safety_node:main',
            'llm_node = doggobot.llm_node:main',
            'pan_node = doggobot.pan_node:main',
        ],
    },
)
