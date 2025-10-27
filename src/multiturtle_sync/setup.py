from setuptools import find_packages, setup

package_name = 'multiturtle_sync'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='samshoni10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'synchronized_drawer = multiturtle_sync.synchronized_drawer:main',
            'drawing_controller = multiturtle_sync.drawing_controller:main',
            'command_sender = multiturtle_sync.command_sender:main',
        ],
    },
)
