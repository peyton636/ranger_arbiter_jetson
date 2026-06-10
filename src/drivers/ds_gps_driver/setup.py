from setuptools import setup

package_name = "ds_gps_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/gps_serial.launch.py",
            "launch/gps_imu.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="ROS2 CH340 GPS NMEA serial driver",
    license="BSD",
    entry_points={
        "console_scripts": [
            "gps_serial = ds_gps_driver.gps_serial_node:main",
        ],
    },
)
