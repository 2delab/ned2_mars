from setuptools import find_packages, setup

package_name = "niryo_ned_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "niryo_ned_vision": ["*.npz"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/vision.launch.py"],
        ),
    ],
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="v",
    maintainer_email="2dellab@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "arucodetection=niryo_ned_vision.arucodetection:main",
            "d435_arucodetection=niryo_ned_vision.d435_arucodetection:main",
            "stereo_calibration=niryo_ned_vision.stereo_distance_calibrator:main",
        ],
    },
)
