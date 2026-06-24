from setuptools import setup

package_name = "eth_gateway"

setup(
    name=package_name,
    version="0.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/eth_gateway.launch.py",
                "launch/cmd_vel_gui.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="Jetson MCU Ethernet UDP BLOB gateway",
    license="MIT",
        entry_points={
        "console_scripts": [
            "eth_gateway = eth_gateway.eth_gateway_node:main",
            "agv_base_eth_bringe = eth_gateway.agv_base_eth_bringe_node:main",
            "cmd_vel_gui = eth_gateway.cmd_vel_gui_node:main",
        ],
    },
)
