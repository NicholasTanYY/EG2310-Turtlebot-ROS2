from setuptools import setup

package_name = 'auto_nav'

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
    maintainer='nus',
    maintainer_email='nus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_course = auto_nav.main:main',
            'set_waypoints = auto_nav.set_waypoints:main',
            'hardcoded_navi_node1 = auto_nav.hardcoded_navi1:main',
            'map2base_node = auto_nav.map2base:main',
            'hardcoded_navi_node2 = auto_nav.hardcoded_navi2:main'
        ],
    },
)
