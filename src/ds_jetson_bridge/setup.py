from setuptools import setup

package_name = "ds_jetson_bridge"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/jetson_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="Jetson to STM32B bridge (protocol V3, 24-byte UART)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "jetson_bridge = ds_jetson_bridge.jetson_bridge_node:main",
            "probe_tty = ds_jetson_bridge.probe_tty:main",
            "sniff_v3 = ds_jetson_bridge.sniff_v3:main",
        ],
    },
)
