from glob import glob

from setuptools import setup

setup(
    name="franka_hand_teleop",
    version="0.1.0",
    packages=["franka_hand_teleop"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/franka_hand_teleop"]),
        ("share/franka_hand_teleop", ["package.xml", "README.md", "REAL_DEPLOYMENT.md"]),
        ("share/franka_hand_teleop/launch", glob("launch/*.launch.py")),
        ("share/franka_hand_teleop/config", glob("config/*")),
        ("share/franka_hand_teleop/urdf", glob("urdf/*")),
        ("share/franka_hand_teleop/tools", glob("tools/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Lab",
    maintainer_email="roboticslab@example.com",
    description="FR3 hand teleoperation with isolated fake and real commissioning paths",
    license="Apache-2.0",
    entry_points={"console_scripts": ["bridge = franka_hand_teleop.bridge:main",
                                     "real_bridge = franka_hand_teleop.real_bridge:main",
                                     "smooth_bridge = franka_hand_teleop.smooth_bridge:main"]},
)
