from setuptools import setup

package_name = "ds_serial_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/serial_monitor.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="STM32 USART3 serial sensor parser",
    license="MIT",
    entry_points={
        "console_scripts": [
            "serial_monitor = ds_serial_monitor.serial_monitor_node:main",
            "find_serial_port = ds_serial_monitor.find_serial_port:main",
        ],
    },
)
