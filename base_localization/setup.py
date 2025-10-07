from setuptools import setup
import os

package_name = 'base_localization'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/base_localization.launch.py']),
        ('share/' + package_name + '/config', ['config/base_localization.yaml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu_email@exemplo.com',
    description='Pose 6D das bases via PnP com intrínsecos vindos de YAML',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_localization = base_localization.base_localization:main',
            'test_marker_pub = base_localization.test_marker_pub:main',
        ],
    },
)
