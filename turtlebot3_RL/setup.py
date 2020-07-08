from setuptools import setup

package_name = 'turtlebot3_RL'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hossein',
    maintainer_email='hsa150@sfu.ca',
    description='Turtlebot3 Reinforcement learning package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_gazebo = turtlebot3_RL.turtlebot3_gazebo.turtlebot3_gazebo:main',
            'turtlebot3_env = turtlebot3_RL.turtlebot3_env.turtlebot3_env:main',
            'turtlebot3_dqn_agent = turtlebot3_RL.turtlebot3_dqn_agent.turtlebot3_dqn_agent:main',
        ],
    },
)
