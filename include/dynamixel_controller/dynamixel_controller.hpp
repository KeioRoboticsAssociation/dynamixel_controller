#ifndef DYNAMIXEL_CONTROLLER_HPP_
#define DYNAMIXEL_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_controller/msg/dynamixel_command.hpp"
#include "dynamixel_controller/msg/dynamixel_response.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

class DynamixelController : public rclcpp::Node {
public:
    DynamixelController();
    ~DynamixelController();

private:
    // Instruction callback: ROS2 で受信した命令に応じた処理を実行する
    void instruction_callback(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg);
    // 指定命令に対する応答データを publish する
    void publish_response(uint8_t instruction_code, const std::vector<uint8_t> & ids, const std::vector<uint8_t> & errors, const std::vector<uint8_t> & data);

    // Command handlers for Dynamixel protocol
    void handle_ping_command(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg);
    void handle_read_command(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg);
    void handle_write_command(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg);
    void handle_sync_read_command(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg);
    void handle_sync_write_command(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg);

    // ROS インターフェース
    rclcpp::Subscription<dynamixel_controller::msg::DynamixelCommand>::SharedPtr instruction_subscriber_;
    rclcpp::Publisher<dynamixel_controller::msg::DynamixelResponse>::SharedPtr response_publisher_;

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
