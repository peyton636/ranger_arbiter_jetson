from setuptools import setup

package_name = "gps_rs232_to_fix"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/gps_rs232_to_fix.launch.py",
            "launch/jetson_rs232_gps_imu_fusion.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="STM32 RS232 GPS to NavSatFix",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gps_rs232_to_fix = gps_rs232_to_fix.gps_rs232_to_fix_node:main",
        ],
    },
)
