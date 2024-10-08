import glob
import os

from setuptools import find_packages, setup

package_name = "default_whoami"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (os.path.join("share", package_name, os.path.dirname(p)), [p])
        for p in glob.glob("description/**/*", recursive=True)
        if os.path.isfile(p)
    ]
    + [
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mmajek",
    maintainer_email="maciej.majek@robotec.ai",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
