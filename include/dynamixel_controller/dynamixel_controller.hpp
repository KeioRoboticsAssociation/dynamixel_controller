#ifndef DYNAMIXEL_CONTROLLER_HPP_
#define DYNAMIXEL_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_controller/msg/dynamixel_controller.hpp"

namespace instruction {
    enum {
        PING         = 0x01,
        READ_DATA    = 0x02,
        WRITE_DATA   = 0x03,
        REG_WRITE    = 0x04,
        ACTION       = 0x05,
        FACTORY_RESET= 0x06,
        REBOOT       = 0x08,
        SYNC_READ    = 0x82,
        SYNC_WRITE   = 0x83,
        BULK_READ    = 0x92,
        BULK_WRITE   = 0x93
    };
}

namespace address {
    enum {
        // EEPROM領域（読み出し専用または読み書き）
        MODEL_NUMBER            = 0x00,   // 2バイト, 読み出し専用
        MODEL_INFORMATION       = 0x02,   // 4バイト, 読み出し専用
        FIRMWARE_VERSION        = 0x06,   // 1バイト, 読み出し専用
        ID                      = 0x07,   // 1バイト, 読み/書き
        BAUD_RATE               = 0x08,   // 1バイト, 読み/書き
        RETURN_DELAY_TIME       = 0x09,   // 1バイト, 読み/書き
        DRIVE_MODE              = 0x0A,   // 1バイト, 読み/書き
        OPERATING_MODE          = 0x0B,   // 1バイト, 読み/書き
        SECONDARY_ID            = 0x0C,   // 1バイト, 読み/書き
        PROTOCOL_TYPE           = 0x0D,   // 1バイト, 読み/書き

        HOMING_OFFSET           = 0x14,   // 4バイト, 読み/書き
        MOVING_THRESHOLD        = 0x18,   // 4バイト, 読み/書き
        TEMPERATURE_LIMIT       = 0x1F,   // 1バイト, 読み/書き
        MAX_VOLTAGE_LIMIT       = 0x20,   // 2バイト, 読み/書き
        MIN_VOLTAGE_LIMIT       = 0x22,   // 2バイト, 読み/書き
        PWM_LIMIT               = 0x24,   // 2バイト, 読み/書き
        CURRENT_LIMIT           = 0x26,   // 2バイト, 読み/書き
        VELOCITY_LIMIT          = 0x28,   // 4バイト, 読み/書き
        MAX_POSITION_LIMIT      = 0x2C,   // 4バイト, 読み/書き
        MIN_POSITION_LIMIT      = 0x30,   // 4バイト, 読み/書き

        // RAM領域
        TORQUE_ENABLE           = 0x40,   // 1バイト, 読み/書き
        LED                     = 0x41,   // 1バイト, 読み/書き
        STATUS_RETURN_LEVEL     = 0x44,   // 1バイト, 読み/書き
        REGISTERED_INSTRUCTION  = 0x45,   // 1バイト, 読み出し専用
        HARDWARE_ERROR_STATUS   = 0x46,   // 1バイト, 読み出し専用
        VELOCITY_I_GAIN         = 0x4C,   // 2バイト, 読み/書き
        VELOCITY_P_GAIN         = 0x4E,   // 2バイト, 読み/書き
        POSITION_D_GAIN         = 0x50,   // 2バイト, 読み/書き
        POSITION_I_GAIN         = 0x52,   // 2バイト, 読み/書き
        POSITION_P_GAIN         = 0x54,   // 2バイト, 読み/書き
        FEEDFORWARD_2_GAIN      = 0x58,   // 2バイト, 読み/書き
        FEEDFORWARD_1_GAIN      = 0x5A,   // 2バイト, 読み/書き
        BUS_WATCHDOG            = 0x62,   // 1バイト, 読み/書き
        GOAL_PWM                = 0x64,   // 2バイト, 読み/書き
        GOAL_CURRENT            = 0x66,   // 2バイト, 読み/書き
        GOAL_VELOCITY           = 0x68,   // 4バイト, 読み/書き
        PROFILE_ACCELERATION    = 0x6C,   // 4バイト, 読み/書き
        PROFILE_VELOCITY        = 0x70,   // 4バイト, 読み/書き
        GOAL_POSITION           = 0x74,   // 4バイト, 読み/書き
        REALTIME_TICK           = 0x78,   // 2バイト, 読み/書き
        MOVING                  = 0x7A,   // 1バイト, 読み出し専用
        PRESENT_PWM             = 0x7C,   // 2バイト, 読み出し専用
        PRESENT_CURRENT         = 0x7E,   // 2バイト, 読み出し専用
        PRESENT_VELOCITY        = 0x80,   // 4バイト, 読み出し専用
        PRESENT_POSITION        = 0x84,   // 4バイト, 読み出し専用
        PRESENT_INPUT_VOLTAGE   = 0x90,   // 2バイト, 読み出し専用
        PRESENT_TEMPERATURE     = 0x92    // 1バイト, 読み出し専用
    };
}

class DynamixelController : public rclcpp::Node {
public:
    DynamixelController();
    ~DynamixelController();

private:
    /// @brief Callback for received instruction messages.
    void instruction_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    /// @brief Publish response data for the given instruction.
    void publish_response(uint8_t instruction_code, const std::vector<uint8_t> & response);

    // ROS インターフェース
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr instruction_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr response_publisher_;

    // Dynamixel SDK 用オブジェクト
    dynamixel::PortHandler   *port_handler_;
    dynamixel::PacketHandler *packet_handler_;

    // デバイスパラメータ（必要に応じてパラメータ化してください）
    const int dxl_id_ = 1;                // モータID（例：1）
    const double protocol_version_ = 2.0; // プロトコルバージョン
};

#endif // DYNAMIXEL_CONTROLLER_HPP_
