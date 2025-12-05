import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import ollama
from typing import List, Dict


def emit_emotion_message(emotion: str, message: str) -> dict:
    """
    Standardized output: used by the LLM for emotion tagging.
    """
    return {"emotion": emotion, "message": message}


available_functions = {"emit_emotion_message": emit_emotion_message}

class LlamaTextSession:
    def __init__(self, model_name='llama3.2:3b'):

        self.model = model_name
        self.messages = [
            {
                "role": "system",
                "content": (
                    "You must ALWAYS call the tool 'emit_emotion_message'. "
                    "Choose an emotion: felicidad, sorpresa, tristeza, enojo. "
                    "Then generate an empathic response in spanish, no questions. "
                    "NEVER respond with plain text. ALWAYS call the tool."
                )
            }
        ]

    def _process_tool_call(self, response):
        """Executes the tool call coming from the model."""
        if not response.message.tool_calls:
            return None

        call = response.message.tool_calls[0]
        func = available_functions.get(call.function.name)
        result = func(**call.function.arguments)

        # Send result back to the model
        self.messages.append({"role": "tool", "content": str(result)})

        return result

    # -------------------------------------------------

    def process_text(self, input_text: str):
        """MAIN function: sends text to the LLM and returns emotion + message."""

        self.messages.append({"role": "user", "content": input_text})

        response = ollama.chat(
            model=self.model,
            messages=self.messages,
            tools=[emit_emotion_message]
        )

        result = self._process_tool_call(response)

        if result is None:
            return ["off", "Error: tool not called"]

        # Commit assistant message to dialogue
        self.messages.append({"role": "assistant", "content": result["message"]})

        return [result["emotion"], result["message"]]


class LlamaEmotionNode(Node):

    def __init__(self):
        super().__init__('llama_emotion_node')

        # Subscribe to text input
        self.subscription = self.create_subscription(
            String,
            '/input_text',
            self.listener_callback,
            10
        )

        # Publisher for emotion
        self.publisher_emotion = self.create_publisher(
            String,
            '/emotion_action',
            10
        )

        # Publisher for message
        self.publisher_message = self.create_publisher(
            String,
            '/message_output',
            10
        )

        # Llama session
        self.llm = LlamaTextSession()

        self.get_logger().info("Llama Emotion Node started.")

    # ----------------------------------------------------------------------

    def listener_callback(self, msg: String):
        """Called whenever new text arrives in /input_text"""

        user_text = msg.data
        self.get_logger().info(f"Received: {user_text}")

        # Process with LLM
        emotion, answer = self.llm.process_text(user_text)

        ros_emotion = String()
        ros_emotion.data = emotion
        ros_msg = String()
        ros_msg.data = answer

        # Publish result
        self.publisher_emotion.publish(ros_emotion)
        self.publisher_message.publish(ros_msg)

        self.get_logger().info(f"Published: {emotion}|{answer}")


def main(args=None):
    rclpy.init(args=args)

    node = LlamaEmotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
