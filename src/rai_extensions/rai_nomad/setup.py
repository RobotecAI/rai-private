# Copyright (C) 2024 Robotec.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

from setuptools import find_packages, setup

package_name = "rai_nomad"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["resource/nomad_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kdabrowski",
    maintainer_email="kacper.dabrowski@robotec.ai",
    description="Package enabling exploration using NoMaD model in RAI",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["nomad = rai_nomad.nomad:main"],
    },
)
