from setuptools import setup
from glob import glob
import os

setup(
    name='potential_fields',
    version='0.0.1',
    packages=['potential_fields'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'potential_fields']),
        ('share/' + 'potential_fields', ['package.xml']),
        (os.path.join('share', 'potential_fields', 'launch'), glob('launch/*.py')),
        (os.path.join('share', 'potential_fields', 'maps'), glob('maps/*')),
        (os.path.join('share', 'potential_fields', 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Teo',
    maintainer_email='teo.swarm.project@proton.me',
    description='Potencijalna polja u Stageu',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pf_controller = potential_fields.pf_controller:main',
            'pf_prikaz_pub = potential_fields.pf_prikaz_pub:main'
        ],
    },
)
