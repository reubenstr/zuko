import os
from glob import glob
from setuptools import setup

package_name = 'quad_main'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),      
        (os.path.join('share', package_name, 'urdf'), glob(package_name + '/src/urdf/*.urdf')),
        (os.path.join('share', package_name, 'stl'), glob(package_name + '/src/stl/*.stl')),
        
        (os.path.join('lib', package_name, 'src'), glob(package_name + '/src/*.py')), 
        (os.path.join('lib', package_name, 'stl'), glob(package_name + '/src/stl/*.stl')),
        (os.path.join('lib', package_name, 'urdf'), glob(package_name + '/src/urdf/*.urdf')),
        (os.path.join('lib', package_name, 'urdf'), glob(package_name + '/src/urdf/*.*')),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reubenstr',
    maintainer_email='reubenstrangelove@gmail.com',
    description='Zuko (quadruped robot) main code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quad_main = quad_main.quad_main:main'            
        ],
    },
)