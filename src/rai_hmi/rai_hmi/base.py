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

from enum import Enum
from typing import List, Optional, Tuple, cast

import rclpy
from ament_index_python.packages import get_package_share_directory
from langchain_community.vectorstores import FAISS
from langchain_core.documents import Document
from langchain_core.tools import BaseTool
from langchain_openai import OpenAIEmbeddings
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rai_hmi.action_handler_mixin import TaskActionMixin
from rai_hmi.tools import QueryDatabaseTool, QueueTaskTool


class HMIStatus(Enum):
    WAITING = "waiting"
    PROCESSING = "processing"


class BaseHMINode(TaskActionMixin):
    """
    Base class for Human-Machine Interface (HMI) nodes in a robotic system.

    Provides core functionality for:
    - Querying a FAISS index to retrieve relevant documents.
    - Publishing the node's processing status ('WAITING' or 'PROCESSING').
    - Handling feedback requests via a rai_interfaces.srv.Feedback service, with an abstract method
      `handle_feedback_request` to be implemented by subclasses.

    Methods:
        query_faiss_index_with_scores: Searches the FAISS index and returns document-score pairs.
        status_callback: Publishes the current processing status.
        feedback_request_callback: Handles incoming feedback service requests.

    Abstract Method:
        handle_feedback_request: Must be implemented by subclasses to process feedback queries.

    Initialization:
        _initialize_system_prompt: Sets up the system prompt based on the robot's identity and constitution.
        _load_documentation: Loads the FAISS index from the robot description package.
    """

    def __init__(self, node_name: str, robot_description_package: Optional[str] = None):
        super().__init__(node_name=node_name)

        if robot_description_package is None:
            self.declare_parameter("robot_description_package", "")
            self.robot_description_package = cast(
                str,
                (
                    self.get_parameter("robot_description_package")
                    .get_parameter_value()
                    .string_value
                ),  # type: ignore
            )
        else:
            self.robot_description_package = robot_description_package

        self.processing = False

        self.create_timer(0.01, self.status_callback)

        self.status_publisher = self.create_publisher(String, "~/status", 10)  # type: ignore
        self.task_addition_request_publisher = self.create_publisher(
            String, "task_addition_requests", 10
        )

        self.constitution_service = self.create_client(
            Trigger,
            "rai_whoami_constitution_service",
        )
        self.identity_service = self.create_client(
            Trigger, "rai_whoami_identity_service"
        )

        self.agent = None
        # order of the initialization is important
        self.system_prompt = self._initialize_system_prompt()
        self.faiss_index = self._load_documentation()
        self.tools = self._initialize_available_tools()
        self.initialize_task_action_client_and_server()

        self.get_logger().info("HMI Node has been started")

    def _initialize_available_tools(self):
        """Initialize common tools for the HMI node."""
        tools: List[BaseTool] = []
        if self.faiss_index is not None:
            tools.append(
                QueryDatabaseTool(get_response=self.query_faiss_index_with_scores)
            )
        tools.append(QueueTaskTool(add_task=self.add_task_to_queue))
        return tools

    def status_callback(self):
        status = HMIStatus.PROCESSING if self.processing else HMIStatus.WAITING
        self.status_publisher.publish(String(data=status.value))

    def query_faiss_index_with_scores(
        self, query: str, k: int = 4
    ) -> List[Tuple[Document, float]]:
        output = self.faiss_index.similarity_search_with_score(query, k)
        return output

    def _initialize_system_prompt(self):
        while not self.constitution_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Constitution service of rai_whoami not available, waiting..."
            )

        while not self.identity_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Identity service of rai_whoami not available, waiting..."
            )

        constitution_request = Trigger.Request()

        constitution_future = self.constitution_service.call_async(constitution_request)
        rclpy.spin_until_future_complete(self, constitution_future)
        constitution_response = constitution_future.result()

        identity_request = Trigger.Request()

        identity_future = self.identity_service.call_async(identity_request)
        rclpy.spin_until_future_complete(self, identity_future)
        identity_response = identity_future.result()

        system_prompt = f"""
        Constitution:
        {constitution_response.message}

        Identity:
        {identity_response.message}

        You are a helpful assistant. You converse with users.
        Assume the conversation is carried over a voice interface, so try not to overwhelm the user.
        If you have multiple questions, please ask them one by one allowing user to respond before
        moving forward to the next question. Keep the conversation short and to the point.
        """

        self.get_logger().info("System prompt initialized!")
        return system_prompt

    def _load_documentation(self) -> Optional[FAISS]:
        try:
            faiss_index = FAISS.load_local(
                get_package_share_directory(self.robot_description_package)
                + "/description",
                OpenAIEmbeddings(),
                allow_dangerous_deserialization=True,
            )
            self.get_logger().info(
                f"FAISS index for {self.robot_description_package} loaded!"
            )
        except (FileNotFoundError, ValueError) as e:
            self.get_logger().error(
                f"Could not load FAISS index from robot description package. Error: \n{e}"
            )
            return None
        return faiss_index
