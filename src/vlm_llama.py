import ollama
import numpy as np
import base64
import io
from PIL import Image

from typing import Union, List

class LlamaVisionSession:
    def __init__(self, model_name='llama3.2-vision'):
        self.model = model_name  # The model remains "loaded" once initialized


    def make_text_request(self, 
                           prompt: str):
        

        response = ollama.chat(
            model=self.model,
            messages=[{
                'role': 'user',
                'content': prompt,
            }]
        )
        return response['message']['content']


    def make_image_request(self, 
                           image: Union[str, np.ndarray, Image.Image],
                           prompt: str):
        

        base64_img = self.encode_image(image)

        response = ollama.chat(
            model=self.model,
            messages=[{
                'role': 'user',
                'content': prompt,
                'images': [base64_img],
            }], 
        )
        # for chunk in response:
        #     print(chunk['message']['content'], end='', flush=True)
        return response['message']['content']
    

    @staticmethod
    def encode_image(image: Union[str, np.ndarray, Image.Image]) -> str:
        """
        Encodes an image (file path, NumPy array, or PIL Image) into a base64 string.
        """ 
        if isinstance(image, str):  # File path
            with open(image, "rb") as img_file:
                encoded_img = base64.b64encode(img_file.read()).decode("utf-8")
        
        elif isinstance(image, Image.Image):  # PIL image
            buffer = io.BytesIO()
            image.save(buffer, format="PNG")  # Ensure a standard format
            encoded_img = base64.b64encode(buffer.getvalue()).decode("utf-8")
        
        elif isinstance(image, np.ndarray):  # NumPy array
            pil_image = Image.fromarray(image)
            buffer = io.BytesIO()
            pil_image.save(buffer, format="PNG")
            encoded_img = base64.b64encode(buffer.getvalue()).decode("utf-8")
        else:
            raise ValueError("Unsupported image format. Use a file path, PIL Image, or NumPy array.")

        return encoded_img



if __name__=="__main__":
    # import time 
    # import cv2 as cv
    # t0 = time.time() 
    # # Create a persistent session
    # llama_session = LlamaVisionSession()
    # print('Load time: ', time.time() - t0)
    # t0 = time.time()

    # # Now you can make multiple calls without reloading the model
    # image_last = '/home/tesistas/Desktop/GONZALO/bags/imgs/fcfm/647_4.png'
    # image_path = "/home/tesistas/Desktop/GONZALO/bags/imgs/fcfm/all_652.png"
    # img = cv.cvtColor(cv.imread(image_path), cv.COLOR_BGR2RGB)
    # img_last = cv.cvtColor(cv.imread(image_last), cv.COLOR_BGR2RGB)
    # imgfull = np.zeros((img.shape[0], img.shape[1]*2, 3), dtype=np.uint8)
    # imgfull[:, :img.shape[1], :] = img_last
    # imgfull[:, img.shape[1]:img.shape[1]*2, :] = img

    # # import matplotlib.pyplot as plt
    # # plt.imshow(imgfull)
    # # plt.show()    
    # # response1 = llama_session.analyze_image(image_path)
    # # print(response1, ' took: ', time.time() - t0, '\n')
    # # t0 = time.time()

    # # response2 = llama_session.analyze_image(image_path, "Describe the objects in detail.")
    # # print(response2, ' took: ', time.time() - t0, '\n')
    # t0 = time.time()

    # response3 = llama_session.make_image_request(imgfull, "You are a mobile robot. You must navigate safely in the environment shown in the image" 
    #                                                     ". There are five labeled trajectories, you must choose the best one (ONLY ONE) that st" 
    #                                                     "ays on paved roads or sidewalks, minimizes bumps and does not crash. The lower number" 
    #                                                     "indicates the closest path to the navigation goal. The First image (left) shows the last trajectory that you chose. The second one (right) is the current fpv image."
    #                                                     "Answer ONLY the best trajectory (no extra text)" 
    #                                                     "as: {trajectory: best_id}. Remember to pick ONLY ONE trajectory")

    # print(response3, ' took: ', time.time() - t0, '\n')
    # time.sleep(10)
    # t0 = time.time()

    # response2 = llama_session.make_image_request(imgfull, "You are a mobile robot. You must navigate safely in the environment shown in the image" 
    #                                                     ". There are five labeled trajectories, you must choose the best one (ONLY ONE) that st" 
    #                                                     "ays on paved roads or sidewalks, minimizes bumps and does not crash. The lower number" 
    #                                                     "indicates the closest path to the navigation goal. The First image (left) shows the last trajectory that you chose. The second one (right) is the current fpv image."
    #                                                     "Answer ONLY the best trajectory (no extra text)" 
    #                                                     "as: {trajectory: best_id}. Remember to pick ONLY ONE trajectory")
    # print(response2, ' took: ', time.time() - t0, '\n')
    # t0 = time.time()

    import ollama

    response = ollama.chat(
        model='llama3.2v-selector',  # Path to your fine-tuned model
        messages=[
            {'role': 'user', 'content': 'Input: Whats the best trajectory? \nOutput: {"trajectory": 4}', 'images': ['/home/tesistas/Desktop/GONZALO/bags/imgs/fcfm/all_652.png']},  # Example input-output pair
            {'role': 'user', 'content': 'Input: Whats the best trajectory? \nOutput: {"trajectory": 3}', 'images': ['/home/tesistas/Desktop/GONZALO/bags/imgs/fcfm/all_100.png']},
            {'role': 'user', 'content': 'Whats the best trajectory?', 'images': ['/home/tesistas/Desktop/GONZALO/bags/imgs/fcfm/all_350.png']}
        ]
    )

    print(response)