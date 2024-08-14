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

import logging
from typing import Any, Dict, List, Literal, Sequence

from langchain.tools import BaseTool
from langchain_core.messages import AIMessage, BaseMessage, ToolCall, ToolMessage

from rai.scenario_engine.messages import ToolMultimodalMessage


def images_to_vendor_format(images: List[str], vendor: str) -> List[Dict[str, Any]]:
    if vendor == "openai":
        return [
            {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{image}",
                },
            }
            for image in images
        ]
    else:
        raise ValueError(f"Vendor {vendor} not supported")


def run_tool_call(
    tool_call: ToolCall,
    tools: Sequence[BaseTool],
) -> Dict[str, Any] | Any:
    logger = logging.getLogger(__name__)
    selected_tool = {k.name: k for k in tools}[tool_call["name"]]

    try:
        if selected_tool.args_schema is not None:
            args = selected_tool.args_schema(**tool_call["args"]).dict()
        else:
            args = dict()
    except Exception as e:
        err_msg = f"Error in preparing arguments for {selected_tool.name}: {e}"
        logger.error(err_msg)
        return err_msg

    logger.info(f"Running tool: {selected_tool.name} with args: {args}")

    try:
        tool_output = selected_tool.run(args)
    except Exception as e:
        err_msg = f"Error in running tool {selected_tool.name}: {e}"
        logger.warning(err_msg)
        return err_msg

    logger.info(f"Successfully ran tool: {selected_tool.name}. Output: {tool_output}")
    return tool_output


def run_requested_tools(
    ai_msg: AIMessage,
    tools: Sequence[BaseTool],
    messages: List[BaseMessage],
    llm_type: Literal["openai", "bedrock"],
):
    internal_messages: List[BaseMessage] = []
    for tool_call in ai_msg.tool_calls:
        tool_output = run_tool_call(tool_call, tools)
        assert isinstance(tool_call["id"], str), "Tool output must have an id."
        if isinstance(tool_output, dict):
            tool_message = ToolMultimodalMessage(
                content=tool_output.get("content", "No response from the tool."),
                images=tool_output.get("images"),
                tool_call_id=tool_call["id"],
            )
            tool_message = tool_message.postprocess(format=llm_type)
        else:
            tool_message = [
                ToolMessage(content=str(tool_output), tool_call_id=tool_call["id"])
            ]
        if isinstance(tool_message, list):
            internal_messages.extend(tool_message)
        else:
            internal_messages.append(tool_message)

    # because we can't answer an aiMessage with an alternating sequence of tool and human messages
    # we sort the messages by type so that the tool messages are sent first
    # for more information see implementation of ToolMultimodalMessage.postprocess

    internal_messages.sort(key=lambda x: x.__class__.__name__, reverse=True)
    messages.extend(internal_messages)
    return messages
