from setuptools import setup

package_name = 'robot_confidence'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/confidence_system.launch.py']),
        ('share/' + package_name + '/config', ['config/confidence_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Robot confidence estimation node and navigation behavior node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'confidence_node = robot_confidence.confidence_node:main',
            'navigation_behavior_node = robot_confidence.navigation_behavior_node:main',
        ],
    },
)
