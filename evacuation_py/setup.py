from setuptools import find_packages, setup

package_name = 'evacuation_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['env/*', 'methods/*', 'test_cases/*', 'utils/*'], 
    },
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
            'main = evacuation_py.main:main'
        ],
    },
)
