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

import os
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from rai.cli.rai_cli import create_rai_ws


@pytest.fixture
def rai_ws(tmp_path: Path):
    # `tmp_path` is a built-in pytest fixture.
    # see: https://docs.pytest.org/en/stable/how-to/tmp_path.html#the-tmp-path-fixture

    mock_parser = MagicMock()
    mock_args = MagicMock()
    mock_args.name = "test_package"
    mock_args.destination_directory = tmp_path
    mock_parser.parse_args.return_value = mock_args

    # Patch argparse.ArgumentParser to return our mock
    with patch("argparse.ArgumentParser", return_value=mock_parser):
        create_rai_ws()

    return tmp_path


def test_create_rai_ws(rai_ws: Path):
    whoami_directory = rai_ws / "test_package_whoami"

    assert os.path.exists(whoami_directory), "Description folder is missing"

    description_files = os.listdir(whoami_directory / "description")
    assert "robot_constitution.txt" in description_files
