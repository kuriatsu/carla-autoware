from setuptools import setup

package_name = 'carla_autoware'

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
    maintainer='kuriatsu',
    maintainer_email='kuriatsubayashi712@gmail.com',
    description='bridge between carla and psim in autoware.universe',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_psim_bridge = scripts.carla_psim_bridge:main',
        ],
    },
)