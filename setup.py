from setuptools import setup

package_name = 'turtle_motion'

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
    maintainer='motaz',
    maintainer_email='motaz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_GoToGoal = turtle_motion.GoToGoal:main',
            'turtle_follower = turtle_motion.turtle_follower:main',
            'turtle_circular_motion = turtle_motion.turtle1_circular_motion:main',
            'turtle_bouncing = turtle_motion.turtlesim_bouncing:main',
        ],
    },
)
