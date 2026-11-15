from setuptools import setup

package_name = 'wall_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CRTA-racer',
    maintainer_email='cirkvencic.karlo@gmail.com',
    description='Wall following with a PID controller, as a step-by-step lesson',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall1 = wall_follow.wall_follow1_beams:main',
            'wall2 = wall_follow.wall_follow2_wall:main',
            'wall3 = wall_follow.wall_follow3_error:main',
            'wall4 = wall_follow.wall_follow4_pid:main',
            'wall5 = wall_follow.wall_follow5_drive:main',
            'wall_follow = wall_follow.wall_follow_node:main',
        ],
    },
)
