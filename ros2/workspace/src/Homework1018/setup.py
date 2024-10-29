from setuptools import find_packages, setup

package_name = 'Homework1018'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='song',
    maintainer_email='songhu42@gmail.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hw_mtpub = Homework1018.hw_mtpub:main", 
            "hw_pub = Homework1018.hw_pub:main", 
            "hw_sub = Homework1018.hw_sub:main", 
            "hw_sub2 = Homework1018.hw_sub2:main", 
            "hw_sub3 = Homework1018.hw_sub3:main"
        ],
    },
)
