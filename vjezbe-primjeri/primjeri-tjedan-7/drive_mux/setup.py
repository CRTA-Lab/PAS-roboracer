from glob import glob

from setuptools import setup

package_name = 'drive_mux'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CRTA-racer',
    maintainer_email='cirkvencic.karlo@gmail.com',
    description='Drive command mux for the simulator: '
                'safety has priority over drive',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_mux = drive_mux.drive_mux:main',
        ],
    },
)
