from glob import glob

from setuptools import setup

package_name = 'gap_follow'

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
    description='Follow the gap, step by step (Lab 4)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap1 = gap_follow.gap_follow1_scan:main',
            'gap2 = gap_follow.gap_follow2_preprocess:main',
            'gap3 = gap_follow.gap_follow3_bubble:main',
            'gap4 = gap_follow.gap_follow4_max_gap:main',
            'gap5 = gap_follow.gap_follow5_best_point:main',
            'gap6 = gap_follow.gap_follow6_drive:main',
            'gap7 = gap_follow.gap_follow7_extend:main',
            'reactive_node = gap_follow.reactive_node:main',
        ],
    },
)
