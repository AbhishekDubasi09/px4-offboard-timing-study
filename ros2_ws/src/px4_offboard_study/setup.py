from setuptools import setup

package_name = 'px4_offboard_study'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhishek Dubasi',
    maintainer_email='dubasiabhishek@gmail.com',
    description='PX4 offboard control nodes and setpoint-stream timing study',
    license='MIT',
    entry_points={
        'console_scripts': [
            'study_node = px4_offboard_study.study_node:main',
            'teleop_node = px4_offboard_study.teleop_node:main',
        ],
    },
)
