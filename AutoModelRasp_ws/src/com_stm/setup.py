from setuptools import find_packages, setup

package_name = 'com_stm'

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
    description='Nodo puente para comunicación serial bidireccional con STM32',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Formato: 'nombre_del_ejecutable = carpeta.archivo:funcion_principal'
            'com_stm = com_stm.com_stm:main',
        ],
    },
)