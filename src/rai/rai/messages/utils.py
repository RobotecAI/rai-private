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

import base64
from typing import Any, Callable, Union

import cv2
import numpy as np
import requests


def preprocess_image(
    image: Union[str, bytes, np.ndarray[Any, np.dtype[np.uint8 | np.float_]]],
    encoding_function: Callable[[Any], str] = lambda x: base64.b64encode(x).decode(
        "utf-8"
    ),
) -> str:
    if isinstance(image, str) and image.startswith(("http://", "https://")):
        response = requests.get(image)
        response.raise_for_status()
        image_data = response.content
    elif isinstance(image, str):
        with open(image, "rb") as image_file:
            image_data = image_file.read()
    elif isinstance(image, bytes):
        image_data = image
        encoding_function = lambda x: x.decode("utf-8")
    elif isinstance(image, np.ndarray):  # type: ignore
        if image.dtype == np.float32 or image.dtype == np.float64:
            image = (image * 255).astype(np.uint8)
        _, image_data = cv2.imencode(".png", image)
        encoding_function = lambda x: base64.b64encode(x).decode("utf-8")
    else:
        image_data = image

    return encoding_function(image_data)
