import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import smbus

class EmotionSender(Node):
    def __init__(self):
        super().__init__('emotion_sender')

        self.bus = smbus.SMBus(1)  # Para Raspberry Pi o Jetson. Usa bus 1
        self.arduino_address = 0x08  # Mismo que en Arduino

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'emotion_angles',  # Debes publicar en este tópico
            self.send_emotion_callback,
            10
        )

    def send_emotion_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn("Se esperaban exactamente 4 ángulos [ceja_izq, ceja_der, oreja_izq, oreja_der]")
            return

        # Convierte a enteros 0-180
        angles = [max(0, min(180, int(a))) for a in msg.data]
        try:
            self.bus.write_i2c_block_data(self.arduino_address, 0, angles)
            self.get_logger().info(f'📤 Enviando ángulos por I2C: {angles}', throttle_duration_sec=0)
        except Exception as e:
            self.get_logger().error(f'❌ Error enviando por I2C: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EmotionSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
