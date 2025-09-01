# setup.py

import os
from glob import glob
from setuptools import setup

package_name = 'robotarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf/mesh'), glob('urdf/mesh/*.stl')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        
        # --- ▼▼▼ 이 부분을 아래와 같이 수정하세요 ▼▼▼ ---
        (os.path.join('share', package_name, 'models', 'robotarm'), glob('models/robotarm/*')),
        # ------------------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mac',
    maintainer_email='jongbob1918@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
        ],
    },
)