from setuptools import setup

package_name = 'yolonas_ros'

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
    maintainer='Miguel Ángel González Santamarta',
    maintainer_email='mgons@unileon.es',
    description='YOLO-NAS for ROS 2',
    license='GPL-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'yolonas_node = yolonas_ros.yolonas_node:main',
        ],
    },
)
