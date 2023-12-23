import os
from glob import glob
from setuptools import setup


package_name = 'rpi_gpio_tank'

setup(
    name=package_name,
    version='0.0.5',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Students, Starting Developer',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Yaboom RaspberryPi GPIO Robot',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'core = ' + package_name + '.core:main',
            'image_processing_face = ' + package_name + '.image_processing_face:main',
        ],
    },
)
