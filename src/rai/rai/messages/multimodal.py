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

from typing import Any, Dict, List, Literal, Optional, TypedDict, Union

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from langchain_core.messages.base import BaseMessage, get_msg_title_repr
from langchain_core.tools import BaseTool


class MultimodalArtifact(TypedDict):
    images: List[str]  # base64 encoded images
    audios: List[str]


class MultimodalMessage(BaseMessage):
    images: Optional[List[str]] = None
    audios: Optional[Any] = None

    def __init__(
        self,
        **kwargs: Any,
    ):
        super().__init__(**kwargs)  # type: ignore

        if self.audios not in [None, []]:
            raise ValueError("Audio is not yet supported")

        _content: List[Union[str, Dict[str, Union[Dict[str, str], str]]]] = []

        if isinstance(self.content, str):
            _content.append({"type": "text", "text": self.content})
        else:
            raise ValueError("Content must be a string")  # for now, to guarantee compat

        if isinstance(self.images, list):
            _image_content = [
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/png;base64,{image}",
                    },
                }
                for image in self.images
            ]
            _content.extend(_image_content)
        self.content = _content

    @property
    def text(self) -> str:
        return self.content[0]["text"]


class HumanMultimodalMessage(HumanMessage, MultimodalMessage):
    def __repr_args__(self) -> Any:
        args = super().__repr_args__()
        new_args = []
        for k, v in args:
            if k == "content":
                v = [c for c in v if c["type"] != "image_url"]
            elif k == "images":
                imgs_summary = [image[0:10] + "..." for image in v]
                v = f'{len(v)} base64 encoded images: [{", ".join(imgs_summary)}]'
            new_args.append((k, v))
        return new_args

    def _no_img_content(self):
        return [c for c in self.content if c["type"] != "image_url"]

    def pretty_repr(self, html: bool = False) -> str:
        title = get_msg_title_repr(self.type.title() + " Message", bold=html)
        # TODO: handle non-string content.
        if self.name is not None:
            title += f"\nName: {self.name}"
        return f"{title}\n\n{self._no_img_content()}"


class SystemMultimodalMessage(SystemMessage, MultimodalMessage):
    pass


class ToolMultimodalMessage(ToolMessage, MultimodalMessage):
    def postprocess(self, format: Literal["openai", "bedrock"] = "openai"):
        if format == "openai":
            return self._postprocess_openai()
        elif format == "bedrock":
            return self._postprocess_bedrock()
        else:
            raise ValueError(f"Unsupported format: {format}")

    def _postprocess_openai(self):
        """OpenAI does not allow images in the tool message.
        Functions dumps the message into human multimodal message and tool message.
        """
        if isinstance(self.images, list):
            human_message = HumanMultimodalMessage(
                content=f"Image returned by a tool call {self.tool_call_id}",
                images=self.images,
                tool_call_id=self.tool_call_id,
            )
            # at this point self.content is a list of dicts
            # we need to extract the text from each dict
            tool_message = ToolMultimodalMessage(
                tool_call_id=self.tool_call_id,
                name=self.name,
                content=" ".join([part.get("text", "") for part in self.content]),
            )
            return [tool_message, human_message]
        else:
            # TODO(maciejmajek): find out if content can be a list
            return ToolMessage(tool_call_id=self.tool_call_id, content=self.content)

    def _postprocess_bedrock(self):
        return self._postprocess_openai()
        # https://github.com/langchain-ai/langchain-aws/issues/75
        # at this moment im not sure if bedrock supports images in the tool message
        content = self.content
        # bedrock expects image and not image_url
        content[1]["type"] = "image"
        content[1]["image"] = content[1].pop("image_url")
        content[1]["image"]["source"] = content[1]["image"].pop("url")

        return ToolMessage(tool_call_id=self.tool_call_id, content=content)


class AiMultimodalMessage(AIMessage, MultimodalMessage):
    pass


class FutureAiMessage:
    """
    Class to represent a response from the AI that is not yet available.
    """

    def __init__(self, tools: List[BaseTool], max_tokens: int = 4096):
        self.tools = tools
        self.max_tokens = max_tokens


class AgentLoop:
    """
    Class to represent a loop of agent actions.
    """

    def __init__(
        self,
        tools: List[BaseTool],
        stop_tool: Optional[str] = None,
        stop_iters: int = 10,
    ):
        self.stop_tool = stop_tool
        self.stop_iters = stop_iters
        if self.stop_tool is not None:
            if not any([tool.__class__.__name__ == stop_tool for tool in tools]):
                raise ValueError("Stop tool not in tools")
        self.tools: List[BaseTool] = tools
