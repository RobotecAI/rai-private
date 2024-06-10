from typing import Any, Dict, List, Optional, Union

from langchain_core.messages import AnyMessage, HumanMessage, ToolMessage


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


class RaiToolMessage(ToolMessage):
    def __init__(
        self, content: str, tool_call_id: str, images: Optional[List[str]] = None
    ):
        self.content = content
        self.images = images
        self.tool_call_id = tool_call_id

    def to_openai(self) -> List[ToolMessage | HumanMessage]:
        if self.images is None:
            return [ToolMessage(content=self.content, tool_call_id=self.tool_call_id)]
        else:
            tool_message = ToolMessage(
                content=self.content, tool_call_id=self.tool_call_id
            )

            human_content = [
                {
                    "type": "text",
                    "text": "Here is the image generated by the last tool call",
                },
            ]
            images_prepared = images_to_vendor_format(self.images, vendor="openai")
            human_content = human_content.extend(images_prepared)
            human_message = HumanMessage(content=human_content)

            return [tool_message, human_message]
