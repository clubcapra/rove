import rclpy
from rclpy.node import Node
from rove_interfaces.srv import Chat


class RoveAI:
    def __init__(self, model_name="tog/TinyLlama-1.1B-alpaca-chat-v1.5"):
        pass

    def predict(self, message, history):
        return "Hello World"


class ChatBot(Node):
    def __init__(self):
        super().__init__('rove_chat')
        self.rove_ai = RoveAI() 
        self.srv = self.create_service(Chat, 'rove_chat',
                                       self.rove_chat_callback)

    def rove_chat_callback(self, request, response):
        message = request.message
        print(f"Received message: {message}")
        response.answer = self.rove_ai.predict(message, history=[])
        return response


def main(args=None):
    rclpy.init(args=args)
    rove_chat = ChatBot()
    rclpy.spin(rove_chat)
    rove_chat.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
