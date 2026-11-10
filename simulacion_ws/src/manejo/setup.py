from setuptools import find_packages, setup

package_name = 'manejo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Jason Medina Martinez',
    maintainer_email='jason240208@gmail.com',
    description='Teleoperacion WASD para el carro simulado.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_wasd = manejo.teleop_wasd:main',
        ],
    },
)