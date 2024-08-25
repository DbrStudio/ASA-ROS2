import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class TeleopKeyNode(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.get_logger().info('Teleop Key Node has been started.')
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            key_char = key.char
            self.publisher_.publish(String(data=key_char))
        except AttributeError:
            pass

    def on_release(self, key):
        # Publish stop command when any key is released
        self.publisher_.publish(String(data='stop'))
        if key == keyboard.Key.esc:
            # Stop listener
            return False

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
