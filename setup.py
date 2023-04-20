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
            'run_course = auto_nav.table6:main',
            'set_waypoints = auto_nav.set_waypoints:main',
            'map2base_node = auto_nav.map2base:main',
            'sim_navi = auto_nav.sim_navi:main',
            'actual_navi = auto_nav.actual_navi:main',
            'button_listener = auto_nav.button_listener:main',
            'beep = auto_nav.beep:main',
            'factory_test = auto_nav.factorytest:main'
        ],
    },
)
