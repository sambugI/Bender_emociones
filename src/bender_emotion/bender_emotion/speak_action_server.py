#!/usr/bin/env python3
# File: speak_action_server.py
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from custom_action_interfaces.action import SpeakEmotion
from std_msgs.msg import String
import asyncio
import time

class SpeakEmotionActionServer(Node):
    def __init__(self):
        super().__init__('speak_emotion_action_server')
        
        # Action Server
        self._action_server = ActionServer(
            self,
            SpeakEmotion,
            'speak_emotion',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Publisher hacia /emotion_action
        self.emotion_action_pub = self.create_publisher(String, '/emotion_action', 10)
        
        # Timer para publicar la emoción "hablar" repetidamente
        self.speak_timer = None
        self.is_speaking = False
        
        self.get_logger().info("SpeakEmotionActionServer iniciado. Esperando metas en 'speak_emotion'...")
    
    def goal_callback(self, goal_request):
        """Callback cuando llega un nuevo goal request."""
        duration = goal_request.duration
        
        if duration <= 0:
            self.get_logger().warn(f"⚠️ Duración inválida: {duration}s. Rechazando goal.")
            return GoalResponse.REJECT
        
        if self.is_speaking:
            self.get_logger().warn("⚠️ Ya hay una sesión de habla activa. Rechazando nuevo goal.")
            return GoalResponse.REJECT
        
        self.get_logger().info(f"🎯 Goal aceptado: hablar durante {duration:.2f}s")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Callback cuando se solicita cancelar un goal."""
        self.get_logger().info("🛑 Cancelación de goal solicitada")
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Callback principal del Action Server."""
        duration = goal_handle.request.duration
        start_time = time.time()
        
        self.get_logger().info(f"▶️ Iniciando habla durante {duration:.2f}s")
        self.is_speaking = True
        
        # Publicar emoción "hablar" cada 0.5 segundos
        publish_interval = 0.5  # segundos
        
        try:
            while True:
                # Verificar si fue cancelado
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("❌ Goal cancelado por el usuario")
                    
                    result = SpeakEmotion.Result()
                    result.success = False
                    result.message = "Habla cancelada por el usuario"
                    return result
                
                # Calcular tiempo transcurrido
                elapsed = time.time() - start_time
                
                # Verificar si ya completamos la duración
                if elapsed >= duration:
                    break
                
                # Publicar emoción "hablar"
                emotion_msg = String()
                emotion_msg.data = "hablar"
                self.emotion_action_pub.publish(emotion_msg)
                self.get_logger().debug(f"🗣️ Publicado 'hablar' ({elapsed:.2f}/{duration:.2f}s)")
                
                # Enviar feedback
                feedback_msg = SpeakEmotion.Feedback()
                feedback_msg.elapsed_time = elapsed
                feedback_msg.current_status = f"Hablando... {elapsed:.1f}/{duration:.1f}s"
                goal_handle.publish_feedback(feedback_msg)
                
                # Esperar antes de la siguiente publicación
                await asyncio.sleep(publish_interval)
            
            # Goal completado exitosamente
            goal_handle.succeed()
            
            result = SpeakEmotion.Result()
            result.success = True
            result.message = f"Habla completada exitosamente durante {duration:.2f}s"
            self.get_logger().info(f"✅ {result.message}")
            
            return result
            
        finally:
            self.is_speaking = False

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeakEmotionActionServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("SpeakEmotionActionServer detenido por el usuario.")
    finally:
        if node:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
