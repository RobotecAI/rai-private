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

from dataclasses import dataclass
from typing import Literal

import tomli


@dataclass
class VendorConfig:
    name: str


@dataclass
class ModelConfig:
    simple_model: str
    complex_model: str
    embeddings_model: str


@dataclass
class AWSConfig(ModelConfig):
    region_name: str


@dataclass
class OllamaConfig(ModelConfig):
    base_url: str


@dataclass
class RAIConfig:
    vendor: VendorConfig
    aws: AWSConfig
    openai: ModelConfig
    ollama: OllamaConfig


def load_config() -> RAIConfig:
    with open("config.toml", "rb") as f:
        config_dict = tomli.load(f)
    return RAIConfig(
        vendor=VendorConfig(**config_dict["vendor"]),
        aws=AWSConfig(**config_dict["aws"]),
        openai=ModelConfig(**config_dict["openai"]),
        ollama=OllamaConfig(**config_dict["ollama"]),
    )


def get_llm_model(model_type: Literal["simple_model", "complex_model"]):
    config = load_config()
    vendor = config.vendor.name
    model_config = getattr(config, vendor)

    if vendor == "openai":
        from langchain_openai import ChatOpenAI

        return ChatOpenAI(model=getattr(model_config, model_type))
    elif vendor == "aws":
        from langchain_aws import ChatBedrock

        return ChatBedrock(
            model_id=getattr(model_config, model_type),
            region_name=model_config.region_name,
        )
    elif vendor == "ollama":
        from langchain_ollama import ChatOllama

        return ChatOllama(
            model=getattr(model_config, model_type), base_url=model_config.base_url
        )
    else:
        raise ValueError(f"Unknown LLM vendor: {vendor}")


def get_embeddings_model():
    config = load_config()
    vendor = config.vendor.name
    model_config = getattr(config, vendor)

    if vendor == "openai":
        from langchain_openai import OpenAIEmbeddings

        return OpenAIEmbeddings(model=model_config.embeddings_model)
    elif vendor == "aws":
        from langchain_aws import BedrockEmbeddings

        return BedrockEmbeddings(
            model_id=model_config.embeddings_model, region_name=model_config.region_name
        )
    elif vendor == "ollama":
        from langchain_ollama import OllamaEmbeddings

        return OllamaEmbeddings(
            model=model_config.embeddings_model, base_url=model_config.base_url
        )
    else:
        raise ValueError(f"Unknown embeddings vendor: {vendor}")
