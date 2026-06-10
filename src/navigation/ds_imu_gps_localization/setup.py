from setuptools import setup

package_name = "ds_imu_gps_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/imu_gps_fusion.launch.py",
            "launch/imu_gps_test.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rxp",
    maintainer_email="rxp@todo.todo",
    description="ROS2 IMU/GPS EKF fusion (Python port)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "imu_gps_localization = ds_imu_gps_localization.localization_node:main",
        ],
    },
)
