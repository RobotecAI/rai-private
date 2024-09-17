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

import glob
import importlib
import os
from collections import defaultdict
from typing import Dict

import pytest
from _pytest.terminal import TerminalReporter
from tabulate import tabulate

from rai.config.models import BEDROCK_CLAUDE_HAIKU, OPENAI_MINI


@pytest.fixture
def rai_python_modules():
    packages = glob.glob("src/*")
    package_names = [os.path.basename(p) for p in packages]
    ros2_python_packages = []
    for package_path, package_name in zip(packages, package_names):
        if os.path.isdir(f"{package_path}/{package_name}"):
            ros2_python_packages.append(package_name)

    return [importlib.import_module(p) for p in ros2_python_packages]


@pytest.fixture
def chat_openai_multimodal():
    from langchain_openai.chat_models import ChatOpenAI

    return ChatOpenAI(**OPENAI_MINI)


@pytest.fixture
def chat_openai_text():
    from langchain_openai.chat_models import ChatOpenAI

    return ChatOpenAI(**OPENAI_MINI)


@pytest.fixture
def chat_bedrock_multimodal():
    from langchain_aws.chat_models import ChatBedrock

    return ChatBedrock(**BEDROCK_CLAUDE_HAIKU)  # type: ignore[arg-missing]


class UsageTracker:
    def __init__(self):
        self.vendor_cost: Dict[str, float] = defaultdict(lambda: 0.0)
        self.vendor_total_tokens: Dict[str, int] = defaultdict(lambda: 0)
        self.vendor_input_tokens: Dict[str, int] = defaultdict(lambda: 0)
        self.vendor_output_tokens: Dict[str, int] = defaultdict(lambda: 0)
        self.total_cost = 0

    def add_usage(
        self,
        vendor: str,
        cost: float,
        total_tokens: int = 0,
        input_tokens: int = 0,
        output_tokens: int = 0,
    ) -> None:
        self.vendor_cost[vendor] += cost
        self.vendor_total_tokens[vendor] += total_tokens
        self.vendor_input_tokens[vendor] += input_tokens
        self.vendor_output_tokens[vendor] += output_tokens

    def get_vendor_cost(self, vendor: str) -> float:
        return self.vendor_cost[vendor]

    def get_total_cost(self) -> float:
        return sum(self.vendor_cost.values())

    def get_vendor_total_tokens(self, vendor: str) -> int:
        return self.vendor_total_tokens[vendor]

    def get_total_tokens(self) -> int:
        return sum(self.vendor_total_tokens.values())

    def get_vendor_input_tokens(self, vendor: str) -> int:
        return self.vendor_input_tokens[vendor]

    def get_total_input_tokens(self) -> int:
        return sum(self.vendor_input_tokens.values())

    def get_vendor_output_tokens(self, vendor: str) -> int:
        return self.vendor_output_tokens[vendor]

    def get_total_output_tokens(self) -> int:
        return sum(self.vendor_output_tokens.values())


tracker = UsageTracker()


@pytest.fixture
def usage_tracker():
    return tracker


def pytest_terminal_summary(
    terminalreporter: TerminalReporter, exitstatus: int, config: pytest.Config
) -> None:
    terminalreporter.section("Usage Summary")
    table = [
        [vendor, f"{cost:.4f}", total_tokens, input_tokens, output_tokens]
        for vendor, cost, total_tokens, input_tokens, output_tokens in zip(
            tracker.vendor_cost.keys(),
            tracker.vendor_cost.values(),
            tracker.vendor_total_tokens.values(),
            tracker.vendor_input_tokens.values(),
            tracker.vendor_output_tokens.values(),
        )
    ]
    table.append(
        [
            "Total",
            f"{tracker.get_total_cost():.4f}",
            tracker.get_total_tokens(),
            tracker.get_total_input_tokens(),
            tracker.get_total_output_tokens(),
        ]
    )
    terminalreporter.write(
        f"{tabulate(table, headers=['Vendor', 'Cost $', 'Total Tokens', 'Input Tokens', 'Output Tokens'], tablefmt='grid')}\n"
    )


def pytest_configure(config: pytest.Config) -> None:
    config.pluginmanager.register(tracker, "usage_tracker")
