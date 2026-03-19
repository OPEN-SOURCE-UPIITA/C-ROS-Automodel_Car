from setuptools import find_packages, setup

package_name = 'manual'

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
    description='Paquete de control manual: Teclado, Joystick y Rutinas de prueba',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'nombre_ejecutable = paquete.archivo:funcion_main'
            'key_control = manual.key_control:main',
            'joy_control = manual.joystick_control:main',
            'full_test = manual.full_test:main',
        ],
    },
)