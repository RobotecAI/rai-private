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
import operator
from typing import Annotated, List, Type, TypedDict

from langchain.agents.agent import AgentExecutor
from langchain.agents.tool_calling_agent.base import create_tool_calling_agent
from langchain_community.vectorstores import FAISS
from langchain_core.documents import Document
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage
from langchain_core.prompts.chat import ChatPromptTemplate
from langchain_core.tools import BaseTool
from langchain_core.vectorstores import VectorStore
from langchain_openai import OpenAIEmbeddings
from langgraph.graph import StateGraph
from pydantic import BaseModel, Field

from rai.apps.document_loader import ingest_documentation

logging.basicConfig(level=logging.WARN)


class QueryDocsToolInput(BaseModel):
    query: str = Field(
        ..., description="The query string to be searched in the documentation."
    )


class QueryDocsTool(BaseTool):
    name: str = "query_docs"
    description: str = "Query the similarity search database."

    args_schema: Type[QueryDocsToolInput] = QueryDocsToolInput

    vector_store: VectorStore = Field(
        ..., description="The vector store to be searched."
    )
    k: int = Field(3, description="The number of results to return.")

    def _run(self, query: str) -> List[Document]:
        return self.vector_store.similarity_search(query, k=self.k)


class State(TypedDict):
    messages: Annotated[list[BaseMessage], operator.add]


def talk_to_docs(documentation_root: str, llm: BaseChatModel):
    docs = ingest_documentation(documentation_root)
    vector_store = FAISS.from_documents(docs, OpenAIEmbeddings())

    query_docs = QueryDocsTool(vector_store=vector_store, k=3)

    prompt = ChatPromptTemplate.from_template(
        "You are a robot called with access to your documentation. "
        "You use a lot of emojis when responding to users. "
        "You always respond in first person. "
        "Your main goal is to help the user find the information they need."
        "Your access to the documentation is not important to the user. "
        "You do not have to inform the user about it unless you did not find needed information."
        "Here is the conversation history: {messages} {agent_scratchpad}"
    )

    agent = create_tool_calling_agent(llm, [query_docs], prompt)  # type: ignore
    agent_executor = AgentExecutor(
        agent=agent, tools=[query_docs], return_intermediate_steps=True  # type: ignore
    )

    def input_node(state: State) -> State:
        user_message = HumanMessage(content=input("You: "))
        return {"messages": [user_message]}

    def agent_node(state: State) -> State:
        messages = state["messages"]
        response = agent_executor.invoke({"messages": messages})
        return {"messages": [AIMessage(content=response["output"])]}

    workflow = StateGraph(State)
    workflow.add_node("user_input", input_node)
    workflow.add_node("agent", agent_node)
    workflow.add_edge("user_input", "agent")
    workflow.add_edge("agent", "user_input")
    workflow.set_entry_point("user_input")

    graph = workflow.compile()
    return graph
