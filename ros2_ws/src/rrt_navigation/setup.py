from setuptools import find_packages, setup

package_name = 'rrt_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rrt_worldlaunch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elizabeth_s',
    maintainer_email='elizabeth_s@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_navigation = rrt_navigation.rrt_navigation:main',
            'test_camera = rrt_navigation.test_camera:main'
        ],
    },
)
