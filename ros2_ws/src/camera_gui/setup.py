from setuptools import find_packages, setup

package_name = 'camera_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/camera_gui/launch', [
            'launch/multi_camera.launch.py',
            'launch/multi_camera_with_sub.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oliver-hankins',
    maintainer_email='oliver.hankins@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_publisher = camera_gui.image_publisher:main',
            'image_subscriber = camera_gui.image_subscriber:main',
            'multi_camera_subscriber = camera_gui.multi_camera_subscriber:main',
        ],
    },
)
