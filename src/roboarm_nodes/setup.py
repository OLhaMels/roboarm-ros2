from setuptools import setup

setup(
    name='roboarm_nodes', version='0.1.0', packages=['roboarm_nodes'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/roboarm_nodes']),
                ('share/roboarm_nodes', ['package.xml'])],
    package_data={'roboarm_nodes': ['web.html']},
    install_requires=['setuptools'], zip_safe=True,
    # colcon test runs pytest only when it is declared here; without it, it
    # falls back to unittest, finds no TestCase classes and fails with code 5.
    extras_require={'test': ['pytest']},
    description='Two-axis robotic arm application nodes',
    entry_points={'console_scripts': [
        'task_manager_node = roboarm_nodes.task_manager:main',
        'camera_node = roboarm_nodes.camera:main',
        'vision_node = roboarm_nodes.vision:main',
        'stream_node = roboarm_nodes.stream:main',
        'web_ui = roboarm_nodes.web:main',
    ]},
)
