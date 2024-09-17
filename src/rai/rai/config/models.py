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

from typing import Literal, TypedDict


class OpenAIKwargs(TypedDict):
    """Dictionary type to specify kwargs for OpenAI models."""

    model: str


class AWSBedrockKwargs(TypedDict):
    """Dictionary type to specify kwargs for AWS Bedrock models."""

    model_id: str
    region_name: Literal["us-west-1", "us-west-2"]


# OpenAI Models
OPENAI_MULTIMODAL: OpenAIKwargs = {
    "model": "gpt-4o-2024-08-06",
}

OPENAI_MINI: OpenAIKwargs = {
    "model": "gpt-4o-mini",
}

# AWS Bedrock Models
BEDROCK_CLAUDE_HAIKU: AWSBedrockKwargs = {
    "model_id": "anthropic.claude-3-haiku-20240307-v1:0",
    "region_name": "us-west-2",
}

BEDROCK_CLAUDE_SONNET: AWSBedrockKwargs = {
    "model_id": "anthropic.claude-3-5-sonnet-20240620-v1:0",
    "region_name": "us-east-1",
}

BEDROCK_CLAUDE_OPUS: AWSBedrockKwargs = {
    "model_id": "anthropic.claude-3-opus-20240229-v1:0",
    "region_name": "us-west-2",
}

BEDROCK_MULTIMODAL: AWSBedrockKwargs = {
    "model_id": "anthropic.claude-3-5-sonnet-20240620-v1:0",
    "region_name": "us-east-1",
}
