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

import argparse
import glob
import logging
import subprocess
from pathlib import Path

import coloredlogs
from langchain_community.vectorstores import FAISS
from langchain_core.messages import SystemMessage

from rai.apps.talk_to_docs import ingest_documentation
from rai.messages import preprocess_image
from rai.messages.multimodal import HumanMultimodalMessage
from rai.utils.model_initialization import get_embeddings_model, get_llm_model

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
coloredlogs.install(level="INFO")  # type: ignore


def parse_whoami_package():
    parser = argparse.ArgumentParser(
        description="Parse robot whoami package. Script builds a vector store and creates a robot identity."
    )
    parser.add_argument(
        "documentation_root", type=str, help="Path to the root of the documentation"
    )
    parser.add_argument(
        "--output",
        type=str,
        required=False,
        default=None,
        help="Path to the output directory",
    )
    args = parser.parse_args()
    save_dir = args.output if args.output is not None else args.documentation_root

    llm = get_llm_model(model_type="simple_model")
    embeddings_model = get_embeddings_model()

    def build_docs_vector_store():
        logger.info("Building the robot docs vector store...")
        faiss_index = FAISS.from_documents(docs, embeddings_model)
        faiss_index.add_documents(docs)
        faiss_index.save_local(save_dir)

    def build_robot_identity():
        logger.info("Building the robot identity...")
        prompt = (
            "You will be given a robot's documentation. "
            "Your task is to identify the robot's identity. "
            "The description should cover the most important aspects of the robot with respect to human interaction, "
            "as well as the robot's capabilities and limitations including sensor and actuator information. "
            "If there are any images provided, make sure to take them into account by thoroughly analyzing them. "
            "Your description should be thorough and detailed."
            "Your reply should start with I am a ..."
        )

        images = glob.glob(args.documentation_root + "/images/*")

        messages = [SystemMessage(content=prompt)] + [
            HumanMultimodalMessage(
                content=documentation,
                images=[preprocess_image(image) for image in images],
            )
        ]
        output = llm.invoke(messages)
        assert isinstance(output.content, str), "Malformed output"

        with open(save_dir + "/robot_identity.txt", "w") as f:
            f.write(output.content)
        logger.info("Done")

    docs = ingest_documentation(
        documentation_root=args.documentation_root + "/documentation"
    )
    documentation = str([doc.page_content for doc in docs])
    n_tokens = len(documentation) // 4.0
    logger.info(
        "Building the robot docs vector store... "
        f"The documentation's length is {len(documentation)} chars, "
        f"approximately {n_tokens} tokens"
    )
    logger.warn("Do you want to continue? (y/n)")
    if input() == "y":
        build_docs_vector_store()
    else:
        logger.info("Skipping the robot docs vector store creation.")

    logger.info(
        "Building the robot identity... "
        f"You can do it manually by creating {save_dir}/robot_identity.txt "
    )
    logger.warn("Do you want to continue? (y/n)")
    if input() == "y":
        build_robot_identity()
    else:
        logger.info(
            f"Skipping the robot identity creation. "
            f"You can do it manually by creating {save_dir}/robot_identity.txt"
        )


def create_rai_ws():
    parser = argparse.ArgumentParser(description="Creation of a robot package.")
    parser.add_argument("--name", type=str, required=True, help="Package's name")
    parser.add_argument(
        "--destination-directory",
        type=str,
        required=False,
        default="rai_ws/",
        help="Package's destination",
    )
    args = parser.parse_args()
    package_name = args.name
    package_destination = args.destination_directory

    subprocess.run(
        [
            "ros2",
            "pkg",
            "create",
            "--build-type",
            "ament_python",
            f"{package_name}_whoami",
            "--destination-directory",
            f"{package_destination}/",
            "--license",
            "Apache-2.0",
        ]
    )

    package_path = Path(f"{package_destination}/{package_name}_whoami/description")
    package_path.mkdir(parents=True, exist_ok=True)

    (package_path / "documentation").mkdir(exist_ok=True)
    (package_path / "images").mkdir(exist_ok=True)
    (package_path / "robot_constitution.txt").touch()

    default_constitution_path = (
        "src/rai/rai/cli/resources/default_robot_constitution.txt"
    )
    with open(default_constitution_path, "r") as file:
        default_constitution = file.read()

    with open(f"{package_path}/robot_constitution.txt", "w") as file:
        file.write(default_constitution)

    # Modify setup.py file
    setup_py_path = Path(f"{package_destination}/{package_name}_whoami/setup.py")
    modify_setup_py(setup_py_path)


# TODO: Refactor this hacky solution
# NOTE (mkotynia) fast solution, worth refactor in the future and testing if it generic for all setup.py file confgurations
def modify_setup_py(setup_py_path: Path):
    with open(setup_py_path, "r") as file:
        setup_content = file.read()

    setup_content = "import glob\nimport os\n\n" + setup_content
    # Find the position to insert the new function
    insert_pos = setup_content.find("setup(")

    # Split the script to insert the new function
    part1 = setup_content[:insert_pos]
    part2 = setup_content[insert_pos:]

    # Adding the new line in the data_files section
    data_files_insert = """
        (os.path.join("share", package_name, os.path.dirname(p)), [p])
        for p in glob.glob("description/**/*", recursive=True)
        if os.path.isfile(p)
    ]
    + ["""

    # Finding the position to insert the new line in the data_files section
    data_files_pos = part2.find("data_files=[\n") + len("data_files=[\n")

    # Splitting the part2 to insert the new line in the data_files section
    part2_head = part2[:data_files_pos]
    part2_tail = part2[data_files_pos:]

    # Reconstruct the modified script
    modified_script = part1 + part2_head + data_files_insert + "\n" + part2_tail

    with open(setup_py_path, "w") as file:
        file.write(modified_script)
