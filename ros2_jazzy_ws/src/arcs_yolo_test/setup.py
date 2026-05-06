from setuptools import find_packages, setup

package_name = 'arcs_yolo_test'

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
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ARCS YOLO Test Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_dummy_pub = arcs_yolo_test.yolo_dummy_pub:main',
            'svo_publisher_opencv = arcs_yolo_test.svo_publisher_opencv:main',
        ],
    },
)
