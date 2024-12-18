from setuptools import find_packages
from setuptools import setup

setup(
    name='rosidl_cli',
    version='4.6.5',
    packages=find_packages(exclude=['test']),
    extras_require={
        'completion': ['argcomplete'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/rosidl_cli',
        ]),
        ('share/rosidl_cli', [
            'package.xml',
            'resource/package.dsv',
        ]),
        ('share/rosidl_cli/environment', [
            'completion/rosidl-argcomplete.bash',
            'completion/rosidl-argcomplete.zsh'
        ]),
    ],
    zip_safe=False,
    author='Michel Hidalgo',
    author_email='michel@ekumenlabs.com',
    maintainer='Aditya Pande, Brandon Ong, Dharini Dutia, Shane Loretz',
    maintainer_email='aditya.pande@openrobotics.org, brandon@openrobotics.org, dharini@openrobotics.org, sloretz@openrobotics.org',  # noqa: E501
    url='https://github.com/ros2/rosidl/tree/master/rosidl_cli',
    download_url='https://github.com/ros2/rosidl/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Command line tools for ROS interface generation.',
    long_description="""\
The tooling provides a single command line script for ROS interface source code generation.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosidl = rosidl_cli.cli:main',
        ],
    }
)
