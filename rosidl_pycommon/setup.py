from setuptools import find_packages
from setuptools import setup

package_name = 'rosidl_pycommon'

setup(
    name=package_name,
    version='4.6.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jacob Perron',
    author_email='jacob@openrobotics.org',
    maintainer='Aditya Pande, Brandon Ong, Dharini Dutia, Shane Loretz',
    maintainer_email='aditya.pande@openrobotics.org, brandon@openrobotics.org, dharini@openrobotics.org, sloretz@openrobotics.org',  # noqa: E501
    description='Common Python functions used by rosidl packages.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
