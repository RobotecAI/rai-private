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

from typing import Any, Type

from langchain_core.tools import BaseTool
from pydantic import BaseModel, Field

from .task import Task


class QueryDatabaseInput(BaseModel):
    query: str = Field(
        ...,
        description=(
            "The query that will be searched in the database."
            " eg: 'PWM information', 'How to use the camera' etc."
        ),
    )


class QueryDatabaseTool(BaseTool):
    name: str = "query_database"
    description: str = "Query the database for information"
    input_type: Type[QueryDatabaseInput] = QueryDatabaseInput
    args_schema: Type[QueryDatabaseInput] = QueryDatabaseInput

    get_response: Any

    def _run(self, query: str):
        retrieval_response = self.get_response(query)
        return str(retrieval_response)


class QueueTaskInput(BaseModel):
    task: Task = Field(..., description="The task to queue")


class QueueTaskTool(BaseTool):
    name: str = "queue_task"
    description: str = "Queue a task for the platform"
    input_type: Type[QueueTaskInput] = QueueTaskInput

    args_schema: Type[QueueTaskInput] = QueueTaskInput

    add_task: Any

    def _run(self, task: Task):
        self.add_task(task)
        return f"Task {task} has been queued for the LLM"
