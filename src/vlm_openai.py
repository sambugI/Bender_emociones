from openai import OpenAI
import base64

import numpy as np
from PIL import Image
import io
import os

from typing import List, Dict, Any, Union, Optional
from vlmsatnav.utils.files import read_yaml, PACKAGE_ROOT


class VLMOpenAI:
    # TODO: add error handling in the response side.

    def __init__(self,
                 api_key: str = None,
                 model: str = 'gpt-4o-mini') -> None:
        
        self._api_key = api_key if api_key else self._autoload_api_key()
        self._model   = model
        
        self._client = OpenAI(api_key=self._api_key)


    def _autoload_api_key(self) -> None:
        
        env_key = os.environ.get("OPENAI_API_KEY")
        if env_key is None:
            # look in the path
            yaml_key = read_yaml(str(PACKAGE_ROOT) + '/vlmsatnav/keys.yaml')['OPENAI_API_KEY']
            if yaml_key is not None:
                print("API key loaded from yaml file.")
                # export the variable
                os.environ["OPENAI_API_KEY"] = yaml_key
                return yaml_key
        else:
            print("API key loaded from environment variable.")
            return env_key
        
        raise ValueError("No API key provided. Please provide an API key or set the OPENAI_API_KEY environment variable.")

    @property
    def model(self) -> str:
        return self._model
    
    @model.setter
    def model(self, model: str) -> None:
        self._model = model


    def make_text_request(self,
                        prompt: str, 
                        context: str = "") -> str:
        
        completion = self._client.chat.completions.create(
            model=self._model,
            messages=[
            {"role": "system", "content": context},
            {"role": "user", "content": prompt},
        ])

        return completion.choices[0].message.content


    def make_image_request(self,
                           image: Union[str, np.ndarray, Image.Image],
                           prompt: str,
                           context: str = "") -> str:
        """
        Makes an API call with an image and a prompt.
        """
        # Encode the image into base64
        encoded_image = self.encode_image(image)
        
        # Prepare the API call
        completion = self._client.chat.completions.create(
            model=self._model,
            messages=[
                {"role": "system", "content": context},
                {"role": "user", "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {
                        "url": f"data:image/png;base64,{encoded_image}", "detail": "low"}}
                ]}
            ],
            temperature=0.0,
        )

        return completion.choices[0].message.content


    @staticmethod
    def encode_image(image: Union[str, np.ndarray, Image.Image]) -> str:
        """
        Encodes an image (file path, NumPy array, or PIL Image) into a base64 string.
        """
        if isinstance(image, str):  # File path
            with open(image, "rb") as img_file:
                encoded = base64.b64encode(img_file.read()).decode("utf-8")
        
        elif isinstance(image, Image.Image):  # PIL image
            buffer = io.BytesIO()
            image.save(buffer, format="PNG")  # Ensure a standard format
            encoded = base64.b64encode(buffer.getvalue()).decode("utf-8")
        
        elif isinstance(image, np.ndarray):  # NumPy array
            pil_image = Image.fromarray(image)
            buffer = io.BytesIO()
            pil_image.save(buffer, format="PNG")
            encoded = base64.b64encode(buffer.getvalue()).decode("utf-8")
        
        else:
            raise ValueError("Unsupported image format. Use a file path, PIL Image, or NumPy array.")
        
        return encoded



if __name__ == '__main__':
    from vlmsatnav.utils.files import read_yaml, PACKAGE_ROOT

    api_key = read_yaml(str(PACKAGE_ROOT) + "/vlmsatnav/keys.yaml")["OPENAI_API_KEY"]
    vlm = VLMOpenAI(api_key=api_key)

    # response = vlm.make_text_request(prompt="what is the capital of Chile")
    response = vlm.make_image_request(image="/home/gonz/Desktop/THESIS/code/global-planning/vlmsat-global-planning/vlmsatnav/satellite_handling/map_with_markers.png", 
                                      prompt="Which of the waypoints requiere special attention?",
                                      context="Consider that a ground mobile robot hast to move through the satellite waypints marked numerically in the images. Answer only the numbers asked")
    print(response)