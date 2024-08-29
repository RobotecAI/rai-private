from setuptools import find_packages, setup

package_name = "led_strip"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wsiekierska",
    maintainer_email="wiktoria.siekierska@robotec.ai",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["led_strip_node = led_strip.led_strip_node:main"],
    },
)
