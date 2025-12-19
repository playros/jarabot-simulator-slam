from setuptools import setup

package_name = 'jarabot_sim'

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
    maintainer='jarabot',
    maintainer_email='jarabot@todo.todo',
    description='Jarabot simulator safety stop node',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'jarabot_sim_safety_stop = jarabot_sim.jarabot_sim_safety_stop:main',
        ],
    },
)
