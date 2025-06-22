#ifndef DYNAMIXEL_CONTROLLER_HPP_
#define DYNAMIXEL_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

class DynamixelController : public rclcpp::Node {
public:
    DynamixelController();
    ~DynamixelController();

private:
    // Instruction callback: ROS2 で受信した命令に応じた処理を実行する
    void instruction_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    // 指定命令に対する応答データを publish する
    void publish_response(uint8_t instruction_code, const std::vector<uint8_t> & response);

    // ROS インターフェース
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr instruction_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr response_publisher_;

    // Dynamixel SDK 用オブジェクト
    dynamixel::PortHandler   *port_handler_;
    dynamixel::PacketHandler *packet_handler_;

    // Bus configuration
    std::unordered_set<uint8_t> ttl_ids_;
    std::unordered_set<uint8_t> rs485_ids_;

    // デバイスパラメータ（例：モータID、プロトコルバージョン）
    const int dxl_id_ = 1;
    const double protocol_version_ = 2.0;
};

#endif // DYNAMIXEL_CONTROLLER_HPP_
