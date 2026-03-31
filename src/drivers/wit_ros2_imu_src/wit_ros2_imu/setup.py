from setuptools import find_packages, setup

package_name = "wit_ros2_imu"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/wit_ros2_imu.launch.py"]),
        ("share/" + package_name + "/config", ["config/wit_ros2_imu.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xinyu Sheng",
    maintainer_email="sheng.xin.yu@faxmail.com",
    description="ROS 2 driver for WIT IMU devices",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["wit_ros2_imu = wit_ros2_imu.wit_ros2_imu:main"],
    },
)
