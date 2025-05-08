from setuptools import find_packages, setup

package_name = 'dynabot_encoders'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igv',
    maintainer_email='igv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "odom = dynabot_encoders.odom:main",
            "reset_encoders = dynabot_encoders.reset_encoders:main",
            "lights = dynabot_encoders.lights:main"
        ],
    },
)