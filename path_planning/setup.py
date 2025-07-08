from setuptools import find_packages, setup
import os

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/' + f for f in os.listdir('launch') if f.endswith('.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yui',
    maintainer_email='yuimomi0513@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning = path_planning.path_planning:main',
            'map_publisher = path_planning.map_publisher:main',
        ],
    },
)
