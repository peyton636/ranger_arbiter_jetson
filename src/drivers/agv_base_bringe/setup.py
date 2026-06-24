from setuptools import setup

package_name = "agv_base_driver"

setup(
    name=package_name,
    version="0.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="AGV base bridge: RS232 lifecycle node + Ethernet UDP BLOB + GUI",
    license="MIT",
)
