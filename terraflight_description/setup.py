from setuptools import find_packages, setup
from glob import glob

package_name = "terraflight_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name,
            [
                "package.xml",
                "launch/show_terraflight.launch.py",
                "urdf/terraflight.urdf.xacro",
            ],
        ),
        ("share/" + package_name + "/meshes/", glob("meshes/*")),
        ("share/" + package_name + "/config/", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Henry Buron",
    maintainer_email="henryburon2024@u.northwestern.edu",
    description="Visualization of the robot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
