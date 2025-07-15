from setuptools import find_packages, setup

package_name = 'py_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # package_data={
    #     '': ['_ydlidar.so'],  # Asegura que se incluya la .so
    # },
    
    # include_package_data=True,

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='h',
    maintainer_email='h@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = py_pub.publisher_member_fuction:main',
                'listener = py_pub.subscriber_member_function:main',
        ],
    },
)
