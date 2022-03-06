import os
from glob import glob
from setuptools import setup

package_name = 'quad_motors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
        (os.path.join('lib', package_name, 'src'), glob(package_name + '/src/*.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devpc',
    maintainer_email='reubenstrangelove@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quad_motors = quad_motors.quad_motors:main'
        ],
    },
)
