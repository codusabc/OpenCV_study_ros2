from setuptools import find_packages, setup
import os
import glob

package_name = 'my_opencv_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.*'))),
        ('share/' + package_name + '/config', glob.glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyjung',
    maintainer_email='cyjung@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'img_pub = my_opencv_tutorials.img_pub:main',
            'convert_hsv = my_opencv_tutorials.convert_hsv:main',
            'edge_detect = my_opencv_tutorials.edge_detect:main',
        ],
    },
)
