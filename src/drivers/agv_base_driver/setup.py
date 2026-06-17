from setuptools import setup

package_name = "agv_base_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/agv_base_driver.launch.py",
            "launch/jetson_rs232_bringup.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="AGV base driver",
    license="MIT",
    entry_points={
        "console_scripts": [
            "agv_base_driver = agv_base_driver.agv_base_driver_node:main",
        ],
    },
)
