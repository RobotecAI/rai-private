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

import base64
import os
from typing import List, Literal, Type, cast

import cv2
import numpy as np
import pytest
from langchain.tools import BaseTool
from langchain_community.callbacks.manager import (
    get_bedrock_anthropic_callback,
    get_openai_callback,
)
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import AIMessage, BaseMessage
from langchain_core.pydantic_v1 import BaseModel, Field
from langfuse.callback import CallbackHandler
from pytest import FixtureRequest

from rai.messages import HumanMultimodalMessage
from rai.tools.utils import run_requested_tools


class GetImageToolInput(BaseModel):
    name: str = Field(..., title="Name of the image")


class GetImageTool(BaseTool):

    name: str = "GetImageTool"
    description: str = "Get an image from the user"

    args_schema: Type[GetImageToolInput] = GetImageToolInput  # type: ignore

    def _run(self, name: str):
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        # convert to png
        image = image.astype(np.uint8)
        _, image = cv2.imencode(".png", image)
        base64_image = base64.b64encode(image).decode("utf-8")
        return {"content": f"Here is the image {name}", "images": [base64_image]}


@pytest.mark.billable
@pytest.mark.parametrize(
    ("llm", "llm_type", "callback"),
    [
        ("chat_openai_multimodal", "openai", get_openai_callback),
        ("chat_bedrock_multimodal", "bedrock", get_bedrock_anthropic_callback),
    ],
)
def test_multimodal_messages(
    llm: BaseChatModel,
    llm_type: Literal["openai", "bedrock"],
    callback,
    usage_tracker,
    request: FixtureRequest,
):
    llm = request.getfixturevalue(llm)  # type: ignore
    tools = [GetImageTool()]
    llm_with_tools = llm.bind_tools(tools)  # type: ignore

    langfuse_handler = CallbackHandler(
        public_key=os.getenv("LANGFUSE_PK"),
        secret_key=os.getenv("LANGFUSE_SK"),
        host=os.getenv("LANGFUSE_HOST"),
        trace_name=request.node.name,
        tags=["test"],
    )

    scenario: List[BaseMessage] = [
        HumanMultimodalMessage(
            content="Can you please describe the contents of test.png image? Remember to use the available tools."
        ),
    ]
    with callback() as cb:
        ai_msg = cast(
            AIMessage,
            llm_with_tools.invoke(scenario, config={"callbacks": [langfuse_handler]}),
        )
        scenario.append(ai_msg)
        scenario = run_requested_tools(ai_msg, tools, scenario, llm_type=llm_type)
        ai_msg = llm_with_tools.invoke(
            scenario, config={"callbacks": [langfuse_handler]}
        )
        usage_tracker.add_usage(
            llm_type,
            cost=cb.total_cost,
            total_tokens=cb.total_tokens,
            input_tokens=cb.prompt_tokens,
            output_tokens=cb.completion_tokens,
        )
