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

import functools
import uuid
from typing import Any, Dict, Optional, Tuple, Type

import rclpy
import rosidl_runtime_py.set_message
import rosidl_runtime_py.utilities
from langchain_core.pydantic_v1 import BaseModel, Field
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from .native import Ros2BaseTool


# --------------------- Inputs ---------------------
class Ros2ActionRunnerInput(BaseModel):
    action_name: str = Field(..., description="Name of the action")
    action_type: str = Field(..., description="Type of the action")
    action_goal_args: Dict[str, Any] = Field(
        ..., description="Dictionary with arguments for the action goal message"
    )


class ActionUidInput(BaseModel):
    uid: str = Field(..., description="Action uid.")


class OptionalActionUidInput(BaseModel):
    uid: Optional[str] = Field(
        None,
        description="Optional action uid. If None - results from all submitted actions will be returned.",
    )


# --------------------- Tools ---------------------
class Ros2GetActionNamesAndTypesTool(Ros2BaseTool):
    name: str = "Ros2GetActionNamesAndTypes"
    description: str = "A tool for getting all ros2 actions names and types"

    def _run(self):
        return self.node.ros_discovery_info.actions_and_types


class Ros2RunAction(Ros2BaseTool):
    name: str = "Ros2ActionRunner"
    description: str = "A tool for running a ros2 action"

    args_schema: Type[Ros2ActionRunnerInput] = Ros2ActionRunnerInput

    def _build_msg(
        self, msg_type: str, msg_args: Dict[str, Any]
    ) -> Tuple[object, Type]:
        """
        Import message and create it. Return both ready message and message class.

        msgs args can have two formats:
        { "goal" : {arg 1 : xyz, ... } or {arg 1 : xyz, ... }
        """

        msg_cls: Type = rosidl_runtime_py.utilities.get_interface(msg_type)
        msg = msg_cls.Goal()

        if "goal" in msg_args:
            msg_args = msg_args["goal"]
        rosidl_runtime_py.set_message.set_message_fields(msg, msg_args)
        return msg, msg_cls

    def _run(
        self, action_name: str, action_type: str, action_goal_args: Dict[str, Any]
    ):
        try:
            goal_msg, msg_cls = self._build_msg(action_type, action_goal_args)
        except Exception as e:
            return f"Failed to build message: {e}"

        client = ActionClient(
            self.node, msg_cls, action_name, callback_group=self.node.callback_group
        )
        client.wait_for_server()
        uid = str(uuid.uuid4())
        future = client.send_goal_async(
            goal_msg,
            feedback_callback=functools.partial(self.node.feedback_callback, uid),
        )
        rclpy.spin_until_future_complete(self.node, future)
        # Calllback names follow official ros2 actions tutorial
        goal_handle: ClientGoalHandle = future.result()  # type: ignore
        if not goal_handle.accepted:
            self.node.get_actions_cache().add_result(uid, "Action rejected")
            return "Goal rejected"

        self.node.get_logger().info("Goal accepted")
        get_result_future = goal_handle.get_result_async()
        self.node.get_actions_cache().register_action(
            uid, action_name, action_type, action_goal_args, get_result_future
        )

        self.node.get_logger().info(f"Action submitted {goal_msg=}")
        return f"Action call uid: {uid}"  # TODO(boczekbartek): maybe refactor to langchain tool call id


class Ros2GetRegisteredActions(Ros2BaseTool):
    name: str = "Ros2GetRegisteredAction"
    description: str = "A tool for checking the results of submitted ros2 actions"

    def _run(self):
        return str(self.node.get_running_actions())


class Ros2CheckActionResults(Ros2BaseTool):
    name: str = "Ros2CheckActionResults"
    description: str = "A tool for checking the results of submitted ros2 actions"

    args_schema: Type[OptionalActionUidInput] = OptionalActionUidInput

    def _run(self, uid: Optional[str] = None):
        return str(self.node.get_results(uid))


class Ros2CancelAction(Ros2BaseTool):
    name: str = "Ros2CancelAction"
    description: str = "Cancel submitted action"

    args_schema: Type[ActionUidInput] = ActionUidInput

    def _run(self, uid: str):
        return str(self.node.cancel_action(uid))


class Ros2ListActionFeedbacks(Ros2BaseTool):
    name = "Ros2ListActionFeedbacks"
    description = "List intermediate feedbacks received during ros2 action. Feedbacks are sent before the action is completed."

    args_schema: Type[OptionalActionUidInput] = OptionalActionUidInput

    def _run(self, uid: Optional[str] = None):
        return str(self.node.get_feedbacks(uid))
