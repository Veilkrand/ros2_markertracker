from setuptools import setup

package_name = 'ros2_markertracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alberto NG',
    maintainer_email='155007+Veilkrand@users.noreply.github.com',
    description='TODO: Package description',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'markertracker = ros2_markertracker.markertracker_node:main'
        ],
    },
)