import rclpy  # ROS2のPythonクライアントライブラリをインポート
from rclpy.node import Node  # ノードクラスをインポート（ROS2の基本単位）
from std_msgs.msg import String  # String型メッセージを使用するためにインポート

# パブリッシャーノードを定義するクラス
class SimplePublisher(Node):
    def __init__(self):
        # Publisherの変数
        node_name  = "simple_publisher"
        topic_name = "chatter" 
        que_size   = 10
        
        '''
        ROS2におけるパブリッシャーのキューサイズは、メッセージのバッファリングに関連しています。
        具体的には、キューサイズは、パブリッシャーがメッセージをパブリッシュする際に、
        送信するメッセージをどれだけの数保持できるかを定義します。
        '''
        
        # 親クラス(Node)のコンストラクタを呼び出し、ノード名を "simple_publisher" に設定
        super().__init__(node_name)
        
        # String型のメッセージを"chatter"トピックに送信するパブリッシャーを作成（キューサイズ10）
        self.pub_       = self.create_publisher(String, topic_name, que_size)
        
        # カウンターの初期値を0に設定
        self.counter_   = 0
        
        # パブリッシュの頻度を1Hzに設定
        self.frequency_ = 1.0
        
        # タイマーを設定し、指定の頻度でタイマーコールバック関数を呼び出す
        self.timer_     = self.create_timer(self.frequency_, self.timerCallback)
        
        # ログにパブリッシュ頻度を出力
        self.get_logger().info("Publishing at %d Hz" % self.frequency_ )
    
    # タイマーがトリガーされたときに呼び出されるコールバック関数
    def timerCallback(self):
        # String型のメッセージを生成
        msg      = String()
        
        # メッセージの内容をカウンターの値とともに設定
        msg.data = "Hello ROS2 - counter: %d" % self.counter_
        
        # メッセージをパブリッシュ
        self.pub_.publish(msg)
        
        # カウンターをインクリメント
        self.counter_ += 1

# メイン関数
def main():
    # rclpyの初期化
    rclpy.init()
    
    # SimplePublisherノードをインスタンス化
    simple_publisher = SimplePublisher()
    
    # ノードをスピンさせ、メッセージのパブリッシュを続ける
    rclpy.spin(simple_publisher)
    
    # ノードを破棄
    simple_publisher.destroy_node()
    
    # rclpyをシャットダウン
    rclpy.shutdown()

# スクリプトが直接実行された場合にメイン関数を呼び出す
if __name__ == "__main__":
    main()
