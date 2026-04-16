from setuptools import find_packages, setup

package_name = 'frontier_exploration_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/exploration.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group 1',
    maintainer_email='picolon@asu.edu',
    description='',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'frontier_explorer_node = frontier_exploration_mapping.frontier_explorer_node:main',
            'semantic_hazard_classifier_node = frontier_exploration_mapping.semantic_hazard_classifier_node:main',
            'behavior_coordinator_node = frontier_exploration_mapping.behavior_coordinator_node:main',
        ],
    },
)
