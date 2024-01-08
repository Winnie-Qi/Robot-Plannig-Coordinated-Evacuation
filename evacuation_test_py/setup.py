from setuptools import find_packages, setup

package_name = 'evacuation_test_py'

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
    maintainer='winnie',
    maintainer_email='winnie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_py = evacuation_test_py.test_py:main'
        ],
    },
)

entry_points={
        'console_scripts': [
                
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
