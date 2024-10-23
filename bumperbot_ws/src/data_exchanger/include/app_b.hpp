#ifndef FLAG_VALUE_EXCHANGE_APP_B_HPP_
#define FLAG_VALUE_EXCHANGE_APP_B_HPP_

#include "rclcpp/rclcpp.hpp"  // ROS2のクライアントライブラリ
#include "std_msgs/msg/int32_multi_array.hpp"  // 複数の整数型メッセージを扱うためのメッセージ型

// AppBクラスの定義。ROS2ノードを継承し、flagとvalueを使用した処理を実装
class AppB : public rclcpp::Node
{
public:
    // コンストラクタの宣言
    AppB();

private:
    // トピックのコールバック関数。メッセージを受信した際に呼ばれる
    void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    // タイマーのコールバック関数。定期的に処理を実行する
    void timer_callback();

    // メッセージを送信するPublisherの定義
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;

    // メッセージを受信するSubscriberの定義
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;

    // 定期的に実行されるタイマーの定義
    rclcpp::TimerBase::SharedPtr timer_;

    // 処理のフラグ
    int flag_;

    // 処理で使う値
    int value_;
};

#endif  // FLAG_VALUE_EXCHANGE_APP_B_HPP_
