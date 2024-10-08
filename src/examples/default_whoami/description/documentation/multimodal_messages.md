# Message Types

RAI implements `MultimodalMessage` that allows using image and audio* information.\
*audio is currently added as a placeholder

## Usage

Use multimodal message via its implementation.

```python
class HumanMultimodalMessage(HumanMessage, MultimodalMessage):
class SystemMultimodalMessage(SystemMessage, MultimodalMessage):
class AiMultimodalMessage(AIMessage, MultimodalMessage):
class ToolMultimodalMessage(ToolMessage, MultimodalMessage):
```

Example:

```python
from rai.messages import HumanMultimodalMessage, preprocess_image
from rai.utils.model_initialization import get_llm_model

base64_image = preprocess_image('https://raw.githubusercontent.com/RobotecAI/RobotecGPULidar/develop/docs/image/rgl-logo.png')

llm = get_llm_model(model_type='complex_model') # initialize your vendor of choice in config.toml
msg = [HumanMultimodalMessage(content='This is an example', images=[base64_image])]
llm.invoke(msg)

# AIMessage(content='The image contains the words "Robotec," "GPU," and "Lidar" written in a stylized,
# colorful font against a black background. The text appears to be composed of red, green, and blue lines that create a 3D effect.'...
```

Implementation of the following messages is identical: HumanMultimodalMessage, SystemMultimodalMessage, AiMultimodalMessage.

ToolMultimodalMessage has an addition of postprocess method, which converts the ToolMultimodalMessage into format that is compatible with a chosen vendor.
