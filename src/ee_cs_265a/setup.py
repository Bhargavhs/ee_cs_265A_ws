import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ee_cs_265a'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bhargav',
    maintainer_email='hsbhargav87@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'global_planner = ee_cs_265a.global_planner:main',
            'pure_pursuit = ee_cs_265a.pure_pursuit:main',
            'trajectory_plotter = ee_cs_265a.trajectory_plotter:main',
            'gt_localizer = ee_cs_265a.gt_localizer:main',
            'dynamic_agent = ee_cs_265a.dynamic_agent:main',
            'local_planner = ee_cs_265a.local_planner:main',
        ],
    },
)
