from setuptools import setup

package_name = 'calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件
        ('share/' + package_name + '/launch', ['launch/joint_calibration.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Python 节点示例包',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 关键配置：定义可执行入口点
    entry_points={
        'console_scripts': [
            'joint_calibration_node = calibration.joint_calibration_node:main'        # 格式：节点名=包.模块:主函数
        ],
    },
)