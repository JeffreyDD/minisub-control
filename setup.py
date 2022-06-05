from setuptools import setup

package_name = 'minisub_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeffrey',
    maintainer_email='jeffreydendrijver@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'imu_tf2_publisher = minisub_control.imu_tf2_publisher:main',
                'imu_raw_tf2_publisher = minisub_control.imu_raw_tf2_publisher:main',
                'twist_thrust_processor = minisub_control.twist_thrust_processor:main',
        ],
    },
)
