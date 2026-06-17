from setuptools import setup

package_name = "rs232_gateway"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/rs232_gateway.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="Jetson RS232 V3 / BLOB v2 gateway",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rs232_gateway = rs232_gateway.rs232_gateway_node:main",
        ],
    },
)
