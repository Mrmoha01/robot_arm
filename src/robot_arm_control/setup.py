from setuptools import find_packages, setup

package_name = "robot_arm_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohammed Al Jaf",
    maintainer_email="noreply@example.com",
    description="Closed-form inverse kinematics and a pick-and-place demo for a 4-joint robot arm.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ik_node = robot_arm_control.ik_node:main",
            "pick_place_demo = robot_arm_control.pick_place_demo:main",
        ],
    },
)
