import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        # Publisherの変数
        node_name  = "simple_subscriber"
        topic_name = "chatter" 
        que_size   = 10
                
        super().__init__(node_name)
        
        self.sub_ = self.create_subscription(String, topic_name, self.msgCallback, que_size)
    
    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)

def main():
    rclpy.init()
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        