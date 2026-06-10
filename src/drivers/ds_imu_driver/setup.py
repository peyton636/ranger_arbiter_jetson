from setuptools import setup

package_name = "ds_imu_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/imu_serial.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="ROS2 WIT IMU serial driver",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wit_imu = ds_imu_driver.wit_imu_node:main",
        ],
    },
)
