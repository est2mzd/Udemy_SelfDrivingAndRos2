#include "app_b.hpp"  // AppBクラスのヘッダファイルをインクルード
#include <chrono>  // タイマー処理に必要なヘッダ
#include <thread>  // スリープ処理に必要なヘッダ

// コンストラクタの実装。初期化と各種設定を行う
AppB::AppB() : Node("app_b"), flag_(0), value_(0)  // ノード名を "app_b" として初期化。flagとvalueも初期化
{
    // Publisherを作成。トピック "topic" でInt32MultiArray型のメッセージを送信する
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("topic", 10);

    // Subscriberを作成。トピック "topic" からInt32MultiArray型のメッセージを受信する
    subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "topic", 10, std::bind(&AppB::topic_callback, this, std::placeholders::_1));

    // 1秒ごとにtimer_callbackを呼び出すタイマーを設定
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&AppB::timer_callback, this));
}

// トピックメッセージを受信するコールバック関数の実装
void AppB::topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    // 受信したflagとvalueを更新
    flag_ = msg->data[0];
    value_ = msg->data[1];
    // 受信したデータをログ出力
    RCLCPP_INFO(this->get_logger(), "AppB Received flag: '%d', value: '%d'", flag_, value_);
}

// タイマーで呼び出されるコールバック関数の実装
void AppB::timer_callback()
{
    // flagが2なら処理を実行
    if (flag_ == 2)
    {
        // valueを+1する処理
        RCLCPP_INFO(this->get_logger(), "AppB Processing: value = value + 1");
        value_ = value_ + 1;
        // 2秒間待機
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // 処理が終わったらflagを1に変更
        flag_ = 1;
        RCLCPP_INFO(this->get_logger(), "AppB Finished Processing: flag = %d, value = %d", flag_, value_);

        // メッセージを作成し、flagとvalueを送信
        auto message = std_msgs::msg::Int32MultiArray();
        message.data.push_back(flag_);
        message.data.push_back(value_);
        publisher_->publish(message);
    }
}

// メイン関数。ノードを起動し、ROS2のスピン処理を行う
int main(int argc, char *argv[])
{
    // ROS2ノードの初期化
    rclcpp::init(argc, argv);
    // AppBのインスタンスを作成してスピン（実行）
    rclcpp::spin(std::make_shared<AppB>());
    // ノードの終了処理
    rclcpp::shutdown();
    return 0;
}
