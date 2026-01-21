from setuptools import find_packages, setup

package_name = 'pi_tests'

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
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "check_temperature = pi_tests.check_temperature:main", 
            "create_folder_server = pi_tests.create_folder_server:main",
            "create_folder_client = pi_tests.create_folder_client:main"
        ],
    },
)
