from setuptools import setup

package_name = 'yasmin_practice_py'
scripts_path = package_name + ".scripts"

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
    maintainer='myuser',
    maintainer_email='imartc04@estudianes.unileon.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ej1 = yasmin_practice_py.ej1:main',
            'ej2 = yasmin_practice_py.ej2:main',
            "ejTopics = yasmin_practice_py.ejTopics:main",
        ],
    },
)
