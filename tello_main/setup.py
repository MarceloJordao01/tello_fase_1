from setuptools import setup

package_name = 'tello_main'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sim.launch.py',
            'launch/real.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/params.yaml',
            'config/rviz.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Controle principal de missão para DJI Tello (sim e real) com TF map→base_link.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = tello_main.navigator_node:main',
        ],
    },
)
