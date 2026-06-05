from setuptools import setup

package_name = "ds_can_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/can_monitor.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="STM32 distance sensor CAN parser and terminal display",
    license="MIT",
    entry_points={
        "console_scripts": [
            "can_monitor = ds_can_monitor.can_monitor_node:main",
        ],
    },
)
