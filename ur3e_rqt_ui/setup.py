from setuptools import setup
import os
from glob import glob

package_name = 'ur3e_rqt_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email',
    description='UR3e RQT UI',
    license='TODO',
    entry_points={
        'rqt_gui_py.plugins': [
            'ur3e_ui = ur3e_rqt_ui.plugin:UR3eUIPlugin',
        ],
    },
)