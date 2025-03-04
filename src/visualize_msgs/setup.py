from setuptools import find_packages, setup

package_name = 'visualize_msgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiayun',
    maintainer_email='lijiayun22@mails.tsinghua.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_marker_node = visualize_msgs.pose_to_marker_node:main',
        ],
    },
)
