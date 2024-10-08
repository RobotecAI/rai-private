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

from pathlib import Path
from typing import Annotated, Dict, List, Optional, Type

import tqdm
from langchain_community.document_loaders import Docx2txtLoader, PyPDFLoader, TextLoader
from langchain_core.document_loaders import BaseLoader
from langchain_core.documents import Document

DEFAULT_SUPPORTED_FILETYPES: List[Annotated[str, 'filetype e.g. ".txt"']] = [
    ".txt",
    ".pdf",
    ".docx",
    ".md",
]

DEFAULT_PARSERS: Dict[Annotated[str, 'filetype e.g. ".txt."'], Type[BaseLoader]] = {
    ".txt": TextLoader,
    ".pdf": PyPDFLoader,
    ".docx": Docx2txtLoader,
    ".md": TextLoader,
}


def load_documents(
    paths: List[Path],
    parsers: Optional[
        Dict[Annotated[str, 'filetype e.g. ".txt."'], Type[BaseLoader]]
    ] = None,
) -> List[Document]:
    documents: List[Document] = []

    if parsers is None:
        parsers = DEFAULT_PARSERS

    for path in tqdm.tqdm(paths, desc="Loading documents"):
        document = parsers[path.suffix](file_path=path).load_and_split()  # type: ignore
        documents.extend(document)

    return documents


def ingest_documentation(
    documentation_root: Path | str, recursive: bool = True
) -> List[Document]:
    documents = find_documents(documentation_root, recursive=recursive)
    return load_documents(documents)


def find_documents(
    path: Path | str,
    filetypes: Optional[List[str]] = None,
    recursive: bool = True,
) -> List[Path]:
    if filetypes is None:
        filetypes = DEFAULT_SUPPORTED_FILETYPES
    documents: List[Path] = []

    path = Path(path)
    finder_function = path.rglob if recursive else path.glob
    for file in finder_function("*"):
        if file.is_file() and file.suffix in filetypes:
            documents.append(file)
    return documents
