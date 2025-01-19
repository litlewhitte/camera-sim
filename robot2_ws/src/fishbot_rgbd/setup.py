from setuptools import setup

package_name = 'fishbot_rgbd'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/config", ['config/camera_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zqh',
    maintainer_email='zqh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node=fishbot_rgbd.camera_save:main'
        ],
    },
)
