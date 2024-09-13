# RAI HMI

The RAI HMI (Human-Machine Interface) allows users to converse with the robot and order new tasks to be added to the queue. Communication topics use plaintext, but can be connected from ASR (Automatic Speech Recognition) and to TTS (Text To Speech) nodes to enable a voice interface.

> **NOTE:** Currently the node is tailored for the Husarion ROSBot XL demo use case. It is expected that it can be generalized to other use cases when `rai_whoami` package is fully developed and integrated.

## ROS 2 Interface

### Subscribed Topics

- **`from_human`** (`std_msgs/String`): Incoming plaintext messages from the user.

### Published Topics

- **`to_human`** (`std_msgs/String`): Outgoing plaintext messages for the user.
- **`task_addition_requests`** (`std_msgs/String`): Tasks to be added to the queue, in JSON format.

## Task JSON Schema

Tasks published on the `task_addition_request` follow the schema below:

```json
{
  "$schema": "http://json-schema.org/draft-04/schema#",
  "type": "object",
  "properties": {
    "name": {
      "type": "string"
    },
    "description": {
      "type": "string"
    },
    "priority": {
      "type": "string",
      "enum": ["highest", "high", "medium", "low", "lowest"]
    },
    "robot": {
      "type": "string"
    }
  },
  "required": ["name", "description", "priority"]
}
```

Optional field `"robot"` is currently not implemented and is intended to be used in a fleet setting, where a specific robot can be requested to perform the task in question.
