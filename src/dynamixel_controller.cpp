#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/group_sync_read.h"
#include "dynamixel_sdk/group_sync_write.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "dynamixel_controller/dynamixel_controller.hpp"
#include "dynamixel_controller/msg/dynamixel_controller.hpp"

// 接続情報のマクロ（必要に応じて調整）
#define BAUDRATE 9600
#define DEVICE_NAME "/dev/ttyUSB"

// メッセージ定義のショートカット
#define MSG dynamixel_controller::msg::DynamixelController

DynamixelController::DynamixelController() : Node("dynamixel_controller") {
    RCLCPP_INFO(this->get_logger(), "DynamixelController node started.");

    // ポートハンドラ、パケットハンドラの初期化
    port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);

    if (!port_handler_->openPort()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", DEVICE_NAME);
    } else {
        RCLCPP_INFO(this->get_logger(), "Port opened: %s", DEVICE_NAME);
    }

    if (!port_handler_->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate: %d", BAUDRATE);
    } else {
        RCLCPP_INFO(this->get_logger(), "Baudrate set: %d", BAUDRATE);
    }

    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

    // ROS2 サブスクライバーの作成 (送信用命令を受け付ける)
    instruction_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "dynamixel_tx", 10,
        std::bind(&DynamixelController::instruction_callback, this, std::placeholders::_1));

    // ROS2 パブリッシャーの作成 (受信応答を publish する)
    response_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("dynamixel_rx", 10);
}

DynamixelController::~DynamixelController() {
    // ポートを閉じ、リソース解放
    port_handler_->closePort();
    delete port_handler_;
    // packet_handler_ はシングルトンのため、削除不要の場合があります。
}

void DynamixelController::publish_response(uint8_t instruction_code, const std::vector<uint8_t> & response) {
    std_msgs::msg::UInt8MultiArray msg;
    // 応答メッセージの先頭に命令コードを入れる
    msg.data.push_back(instruction_code);
    msg.data.insert(msg.data.end(), response.begin(), response.end());
    response_publisher_->publish(msg);
}

void DynamixelController::instruction_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty instruction message.");
        return;
    }

    uint8_t instr = msg->data[0];
    uint8_t dxl_error = 0;
    int comm_result = COMM_TX_FAIL;
    std::vector<uint8_t> response_data;

    switch (instr) {
        case MSG::PING: {
            // PING 命令：モデルナンバーの取得
            uint16_t model_number = 0;
            comm_result = packet_handler_->ping(port_handler_, dxl_id_, &model_number, &dxl_error);
            if (comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Ping failed: %s", packet_handler_->getTxRxResult(comm_result));
            } else {
                RCLCPP_INFO(this->get_logger(), "Ping success, model number: %d", model_number);
                response_data.push_back(static_cast<uint8_t>(model_number & 0xFF));
                response_data.push_back(static_cast<uint8_t>((model_number >> 8) & 0xFF));
            }
            publish_response(instr, response_data);
            break;
        }
        case MSG::READ_DATA: {
            // READ_DATA 命令：指定アドレス・データ長の読み出し
            if (msg->data.size() < 3) {
                RCLCPP_ERROR(this->get_logger(), "READ_DATA instruction requires address and length.");
                break;
            }
            uint8_t read_address = msg->data[1];
            uint8_t read_length  = msg->data[2];
            if (read_length == 1) {
                uint8_t data = 0;
                comm_result = packet_handler_->read1ByteTxRx(port_handler_, dxl_id_, read_address, &data, &dxl_error);
                if (comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "Read1ByteTxRx failed: %s", packet_handler_->getTxRxResult(comm_result));
                } else {
                    RCLCPP_INFO(this->get_logger(), "Read1ByteTxRx success: %d", data);
                    response_data.push_back(data);
                }
            } else if (read_length == 2) {
                uint16_t data = 0;
                comm_result = packet_handler_->read2ByteTxRx(port_handler_, dxl_id_, read_address, &data, &dxl_error);
                if (comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "Read2ByteTxRx failed: %s", packet_handler_->getTxRxResult(comm_result));
                } else {
                    RCLCPP_INFO(this->get_logger(), "Read2ByteTxRx success: %d", data);
                    response_data.push_back(static_cast<uint8_t>(data & 0xFF));
                    response_data.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
                }
            } else if (read_length == 4) {
                uint32_t data = 0;
                comm_result = packet_handler_->read4ByteTxRx(port_handler_, dxl_id_, read_address, &data, &dxl_error);
                if (comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "Read4ByteTxRx failed: %s", packet_handler_->getTxRxResult(comm_result));
                } else {
                    RCLCPP_INFO(this->get_logger(), "Read4ByteTxRx success: %u", data);
                    response_data.push_back(static_cast<uint8_t>(data & 0xFF));
                    response_data.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
                    response_data.push_back(static_cast<uint8_t>((data >> 16) & 0xFF));
                    response_data.push_back(static_cast<uint8_t>((data >> 24) & 0xFF));
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported read length: %d", read_length);
            }
            publish_response(instr, response_data);
            break;
        }
        case MSG::WRITE_DATA: {
            // WRITE_DATA 命令：データ書き込み
            if (msg->data.size() < 3) {
                RCLCPP_ERROR(this->get_logger(), "WRITE_DATA instruction requires address and data.");
                break;
            }
            uint8_t write_address = msg->data[1];
            size_t data_length = msg->data.size() - 2;
            int dxl_comm_result = COMM_TX_FAIL;
            if (data_length == 1) {
                uint8_t value = msg->data[2];
                dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, dxl_id_, write_address, value, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "Write1ByteTxRx failed: %s", packet_handler_->getTxRxResult(dxl_comm_result));
                } else {
                    RCLCPP_INFO(this->get_logger(), "Write1ByteTxRx success: wrote %d", value);
                }
            } else if (data_length == 2) {
                uint16_t value = static_cast<uint16_t>(msg->data[2]) | (static_cast<uint16_t>(msg->data[3]) << 8);
                dxl_comm_result = packet_handler_->write2ByteTxRx(port_handler_, dxl_id_, write_address, value, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "Write2ByteTxRx failed: %s", packet_handler_->getTxRxResult(dxl_comm_result));
                } else {
                    RCLCPP_INFO(this->get_logger(), "Write2ByteTxRx success: wrote %d", value);
                }
            } else if (data_length == 4) {
                uint32_t value = static_cast<uint32_t>(msg->data[2])
                                | (static_cast<uint32_t>(msg->data[3]) << 8)
                                | (static_cast<uint32_t>(msg->data[4]) << 16)
                                | (static_cast<uint32_t>(msg->data[5]) << 24);
                dxl_comm_result = packet_handler_->write4ByteTxRx(port_handler_, dxl_id_, write_address, value, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "Write4ByteTxRx failed: %s", packet_handler_->getTxRxResult(dxl_comm_result));
                } else {
                    RCLCPP_INFO(this->get_logger(), "Write4ByteTxRx success: wrote %u", value);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported write data length: %zu", data_length);
            }
            response_data.push_back(dxl_error);
            publish_response(instr, response_data);
            break;
        }
        case MSG::SYNC_READ: {
            // SYNC_READ 命令:
            // メッセージフォーマット:
            // [SYNC_READ, start_address, data_length, id1, id2, ...]
            if (msg->data.size() < 4) {
                RCLCPP_ERROR(this->get_logger(), "SYNC_READ instruction requires start_address, data_length, and at least one ID.");
                break;
            }
            uint8_t start_address = msg->data[1];
            uint8_t data_length = msg->data[2];
            std::vector<uint8_t> id_list;
            for (size_t i = 3; i < msg->data.size(); i++) {
                id_list.push_back(msg->data[i]);
            }
            // GroupSyncRead オブジェクトの生成
            dynamixel::GroupSyncRead groupSyncRead(port_handler_, packet_handler_, start_address, data_length);
            // 各 ID のパラメータを追加
            for (auto id : id_list) {
                if (!groupSyncRead.addParam(id)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add param for ID: %d", id);
                }
            }
            // 同期読み出し命令の実行
            comm_result = groupSyncRead.txRxPacket();
            if (comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "SYNC_READ failed: %s", packet_handler_->getTxRxResult(comm_result));
            } else {
                // 各 ID からデータを取得して応答データに追加
                for (auto id : id_list) {
                    if (groupSyncRead.isAvailable(id, start_address, data_length)) {
                        for (uint8_t offset = 0; offset < data_length; offset++) {
                            uint8_t data = groupSyncRead.getData(id, start_address, offset);
                            response_data.push_back(data);
                        }
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "SYNC_READ data not available for ID: %d", id);
                    }
                }
                publish_response(instr, response_data);
            }
            break;
        }
        case MSG::SYNC_WRITE: {
            // SYNC_WRITE 命令:
            // メッセージフォーマット:
            // [SYNC_WRITE, start_address, data_length, id1, data_bytes..., id2, data_bytes..., ...]
            if (msg->data.size() < 3) {
                RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE instruction requires start_address, data_length, and data.");
                break;
            }
            uint8_t start_address = msg->data[1];
            uint8_t data_length = msg->data[2];
            size_t expected_length = msg->data.size() - 3;
            if (expected_length % (1 + data_length) != 0) {
                RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE data format error: inconsistent data length.");
                break;
            }
            size_t num_entries = expected_length / (1 + data_length);
            dynamixel::GroupSyncWrite groupSyncWrite(port_handler_, packet_handler_, start_address, data_length);
            size_t index = 3;
            for (size_t i = 0; i < num_entries; i++) {
                uint8_t id = msg->data[index++];
                std::vector<uint8_t> param_data;
                for (uint8_t j = 0; j < data_length; j++) {
                    param_data.push_back(msg->data[index++]);
                }
                if (!groupSyncWrite.addParam(id, param_data.data())) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add param for SYNC_WRITE, ID: %d", id);
                }
            }
            int dxl_comm_result = groupSyncWrite.txPacket();
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE failed: %s", packet_handler_->getTxRxResult(dxl_comm_result));
            } else {
                RCLCPP_INFO(this->get_logger(), "SYNC_WRITE success.");
                response_data.push_back(dxl_error);
                publish_response(instr, response_data);
            }
            break;
        }
        default:
            RCLCPP_WARN(this->get_logger(), "Received unsupported instruction: 0x%02X", instr);
            break;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
