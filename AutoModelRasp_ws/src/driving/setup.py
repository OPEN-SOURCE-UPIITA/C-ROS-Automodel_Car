from setuptools import find_packages, setup

package_name = 'driving'

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
    maintainer='alereyes',
    maintainer_email='alerey.lira@gmail.com',
    description='Paquete de conduccion autonoma: Deteccion de carriles, senales y control PID',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_senales = driving.detector_senales:main',
            'detector_carril = driving.detector_carril:main',
            'drive_carril = driving.drive_carril:main',
        ],
    },
)
