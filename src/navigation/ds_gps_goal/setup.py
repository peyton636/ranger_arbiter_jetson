from setuptools import setup

package_name = "ds_gps_goal"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/gps_goal.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="ROS2 GPS goal to Nav2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gps_goal = ds_gps_goal.gps_goal_node:main",
        ],
    },
)
