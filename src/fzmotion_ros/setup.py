from setuptools import setup

package_name = 'fzmotion_ros'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/fzmotion.yaml']),
        ('share/' + package_name + '/launch', ['launch/fzmotion.launch.py']),
    ],
    install_requires=['setuptools', 'protobuf==3.20.3', 'pyzmq>=25,<28'],
    zip_safe=True,
    maintainer='sym',
    maintainer_email='sym@todo.todo',
    description='Receive FZMotion rigid body pose over LuMoSDK ZMQ and publish ROS 2 topics / tf',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fzmotion_node = fzmotion_ros.fzmotion_node:main',
        ],
    },
)
