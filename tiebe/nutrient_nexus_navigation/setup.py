from glob import glob
from setuptools import find_packages, setup


package_name = 'nutrient_nexus_navigation'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/config', glob('config/*')),
        (f'share/{package_name}/worlds', glob('worlds/*')),
        (f'share/{package_name}/rviz', glob('rviz/*')),
        (f'share/{package_name}/test', glob('test/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tiebie',
    maintainer_email='tiebie@example.com',
    description='ROS2 navigation scaffolding for Nutrient Nexus',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zone_detector = nutrient_nexus_navigation.zone_detector:main',
        ],
    },
)
