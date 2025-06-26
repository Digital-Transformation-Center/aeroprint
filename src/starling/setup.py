from setuptools import setup, find_packages
import os
from glob import glob

package_name = "starling"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ryan",
    maintainer_email="ryankuederle@icloud.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_node = starling.test_node:main",
            "test_pub = starling.test_pub:main",
            "circle-flight = starling.circle_flight:main",
            "custom-pc-pub = starling.custom_pc_publisher:main",
            "helical-flight = starling.helical_flight_node:main",
            "new-helix = starling.modular_helix_node:main",
            "odometry-tf-publisher = starling.odometry_tf_publisher:main",
            "point-cloud-transformer = starling.point_cloud_transformer:main",
            "static-tof-tf-publisher = starling.static_tof_tf_publisher:main",
            "static-world-to-odom-tf-publisher = starling.static_world_to_odom_tf_publisher:main",
        ],
    },
)
