from setuptools import setup

package_name = 'ouster_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/localize_with_bridge.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@example.com',
    description='Bridge: live_pose.csv → /tf (/odom)',
    license='BSD',
    entry_points={
        'console_scripts': [
            'csv_tf_bridge = ouster_bridge.csv_tf_bridge:main',
        ],
    },
)

