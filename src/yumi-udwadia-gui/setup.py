from setuptools import find_packages, setup
import os

package_name = 'yumi_udwadia_gui'

setup(
    name='yumi-udwadia-gui',
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['share/ament_index/resource_index/packages/yumi-udwadia-gui']),
        ('share/' + 'yumi-udwadia-gui', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ismilioshyn@gmail.com',
    description='GUI for sending commands to YumiUdwadiaController',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yumi_gui = yumi_udwadia_gui.yumi_gui:main',
        ],
    },
)
