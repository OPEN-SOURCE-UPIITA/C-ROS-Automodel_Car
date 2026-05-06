from setuptools import find_packages, setup

package_name = 'demo'

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
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #Nodos del paquete, maniobnras no porque solo son para lógica, no son nodos
            'maquina_estados = demo.demo_me:main',
            'vision_rebase = demo.demo_rebase:main',
            'vision_senial = demo.demo_seniales:main',

        ],
    },
)
