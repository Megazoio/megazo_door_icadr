from setuptools import find_packages, setup

package_name = 'door_adapter_megazo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/run.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='megazo_support',
    maintainer_email='hello@megazo.io',
    description='RMF Door Adapter for Megazo ICADR',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'door_adapter = door_adapter_megazo.door_adapter:main'
        ],
    },
)
