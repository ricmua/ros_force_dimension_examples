from setuptools import find_packages, setup

package_name = 'force_dimension_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/example.launch.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a.whit',
    maintainer_email='a@whit.contact',
    description='ROS2 implementations of the examples provided by the Force ' \
               +'Dimension haptics and robotics SDK',
    license='Mozille Public License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': 
          ['segment = force_dimension_examples.segment:entry_point',
           #'gui = force_dimension_examples.console:entry_point',
           'console = force_dimension_examples.console:entry_point',
          ],
    },
)

# https://docs.ros.org/en/iron/Tutorials/Intermediate/Launch/Launch-system.html#creating-the-structure-to-hold-launch-files
