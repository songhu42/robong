from setuptools import find_packages, setup
from glob import glob 
import os 

package_name = 'SimplePkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py')))

#        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='song',
    maintainer_email='songhu42@gmail.com',
    description='TODO: Package description',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hello = SimplePkg.hello:main", 
            "hello_sub = SimplePkg.hello_sub:main",
            "hello_pub = SimplePkg.hello_pub:main", 
            "hw_pub = Homework1018.hw_pub:main", 
            "hw_mtpub = Homework1018.hw_mtpub:main", 
            "hw_sub = Homework1018.hw_sub:main",
            "hw_sub2 = Homework1018.hw_sub2:main",
            "hw_sub3 = Homework1018.hw_sub3:main",
            "move_turtle = SimplePkg.move_turtle:main",
            "second = SimplePkg.second:main"
        ],
    },
)
