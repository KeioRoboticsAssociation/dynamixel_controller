#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/group_sync_read.h"
#include "dynamixel_sdk/group_sync_write.h"
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "dynamixel_controller/dynamixel_controller.hpp"
#include "dynamixel_controller/msg/dynamixel_controller.hpp"

// 接続情報のマクロ（必要に応じて調整）
#define BAUDRATE 1000000
#define DEVICE_NAME "/dev/ttyUSB0"

// メッセージ定義のショートカット
#define MSG dynamixel_controller::msg::DynamixelController

// 例示用：エスケープ処理対象のヘッダーシーケンス
// ※この内容はプロトコルに合わせて変更してください。
static const std::vector<uint8_t> header_sequence = {0x01, 0x02};

//--------------------------------------------------------------------
// Escape function: Parameter 部中に header_sequence と一致するシーケンスが現れた場合
// その末尾に 0xFD を1バイト追加する
std::vector<uint8_t> escapeData(const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> escaped;
    size_t i = 0;
    while (i < data.size()) {
        bool match = false;
        if (i + header_sequence.size() <= data.size()) {
            match = true;
            for (size_t j = 0; j < header_sequence.size(); j++) {
                if (data[i+j] != header_sequence[j]) {
                    match = false;
                    break;
                }
            }
        }
        if (match) {
            // ヘッダーシーケンスそのものをコピー
            for (size_t j = 0; j < header_sequence.size(); j++) {
                escaped.push_back(data[i+j]);
            }
            // その末尾に 0xFD を追加
            escaped.push_back(0xFD);
            i += header_sequence.size();
        } else {
            // ヘッダーと一致しない場合はそのままコピー
            escaped.push_back(data[i]);
            i++;
        }
    }
    return escaped;
}
//--------------------------------------------------------------------

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

    // Load bus configuration from parameters
    std::vector<int64_t> ttl_param;
    std::vector<int64_t> rs485_param;
    this->declare_parameter("ttl_ids", ttl_param);
    this->declare_parameter("rs485_ids", rs485_param);
    this->get_parameter("ttl_ids", ttl_param);
    this->get_parameter("rs485_ids", rs485_param);
    for (auto id : ttl_param) {
        ttl_ids_.insert(static_cast<uint8_t>(id));
    }
    for (auto id : rs485_param) {
        rs485_ids_.insert(static_cast<uint8_t>(id));
    }

    // Debug: Print loaded configuration
    RCLCPP_INFO(this->get_logger(), "=== Bus Configuration Loaded ===");
    RCLCPP_INFO(this->get_logger(), "TTL IDs: ");
    for (auto id : ttl_ids_) {
        RCLCPP_INFO(this->get_logger(), "  TTL ID: %d", id);
    }
    RCLCPP_INFO(this->get_logger(), "RS485 IDs: ");
    for (auto id : rs485_ids_) {
        RCLCPP_INFO(this->get_logger(), "  RS485 ID: %d", id);
    }
    RCLCPP_INFO(this->get_logger(), "================================");

    // ROS2 サブスクライバーの作成 (送信用命令を受け付ける)
    instruction_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "dynamixel_tx", 100,
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

// 修正：送信前に escapeData を実行して、ヘッダーシーケンスと一致する部分をエスケープする
void DynamixelController::publish_response(uint8_t instruction_code, const std::vector<uint8_t> & response) {
    std_msgs::msg::UInt8MultiArray msg;
    // 応答メッセージの先頭に命令コードを入れる
    msg.data.push_back(instruction_code);
    // パラメータ部に対してエスケープ処理を実施
    std::vector<uint8_t> escaped_response = escapeData(response);
    // 必要に応じて、Length や Checksum の再計算もここで行う（本例では割愛）
    msg.data.insert(msg.data.end(), escaped_response.begin(), escaped_response.end());
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
            std::vector<uint8_t> ttl_targets;
            std::vector<uint8_t> rs_targets;
            for (auto id : id_list) {
                if (rs485_ids_.count(id)) {
                    rs_targets.push_back(id);
                } else {
                    ttl_targets.push_back(id);
                }
            }

            std::unordered_map<uint8_t, std::vector<uint8_t>> data_map;
            if (!ttl_targets.empty()) {
                dynamixel::GroupSyncRead ttlRead(port_handler_, packet_handler_, start_address, data_length);
                for (auto id : ttl_targets) {
                    if (!ttlRead.addParam(id)) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to add TTL param for ID: %d", id);
                    }
                }
                comm_result = ttlRead.txRxPacket();
                if (comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "SYNC_READ TTL failed: %s", packet_handler_->getTxRxResult(comm_result));
                } else {
                    for (auto id : ttl_targets) {
                        if (ttlRead.isAvailable(id, start_address, data_length)) {
                            uint32_t data = ttlRead.getData(id, start_address, data_length);
                            std::vector<uint8_t> bytes;
                            for (uint8_t i = 0; i < data_length; i++) {
                                bytes.push_back(static_cast<uint8_t>((data >> (i*8)) & 0xFF));
                            }
                            data_map[id] = bytes;
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "TTL SYNC_READ data not available for ID: %d", id);
                        }
                    }
                }
            }

            if (!rs_targets.empty()) {
                dynamixel::GroupSyncRead rsRead(port_handler_, packet_handler_, start_address, data_length);
                for (auto id : rs_targets) {
                    if (!rsRead.addParam(id)) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to add RS485 param for ID: %d", id);
                    }
                }
                comm_result = rsRead.txRxPacket();
                if (comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "SYNC_READ RS485 failed: %s", packet_handler_->getTxRxResult(comm_result));
                } else {
                    for (auto id : rs_targets) {
                        if (rsRead.isAvailable(id, start_address, data_length)) {
                            uint32_t data = rsRead.getData(id, start_address, data_length);
                            std::vector<uint8_t> bytes;
                            for (uint8_t i = 0; i < data_length; i++) {
                                bytes.push_back(static_cast<uint8_t>((data >> (i*8)) & 0xFF));
                            }
                            data_map[id] = bytes;
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "RS485 SYNC_READ data not available for ID: %d", id);
                        }
                    }
                }
            }

            for (auto id : id_list) {
                auto it = data_map.find(id);
                if (it != data_map.end()) {
                    response_data.insert(response_data.end(), it->second.begin(), it->second.end());
                }
            }

            publish_response(instr, response_data);
            break;
        }
        case MSG::SYNC_WRITE: {
            if (msg->data.size() < 3) {
                RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE instruction requires start_address, data_length, and data.");
                break;
            }
            RCLCPP_INFO(this->get_logger(), "success.");
            uint8_t start_address = msg->data[1];
            uint8_t data_length = msg->data[2];
            size_t expected_length = msg->data.size() - 3;
            if (expected_length % (1 + data_length) != 0) {
                RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE data format error: inconsistent data length.");
                break;
            }
            size_t num_entries = expected_length / (1 + data_length);
            size_t index = 3;
            dynamixel::GroupSyncWrite ttlWrite(port_handler_, packet_handler_, start_address, data_length);
            dynamixel::GroupSyncWrite rsWrite(port_handler_, packet_handler_, start_address, data_length);
            bool ttl_has_param = false;
            bool rs_has_param = false;

            for (size_t i = 0; i < num_entries; i++) {
                uint8_t id = msg->data[index++];
                std::vector<uint8_t> param_data;
                for (uint8_t j = 0; j < data_length; j++) {
                    param_data.push_back(msg->data[index++]);
                }
                if (rs485_ids_.count(id)) {
                    if (!rsWrite.addParam(id, param_data.data())) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to add RS485 param for SYNC_WRITE, ID: %d", id);
                    }
                    rs_has_param = true;
                } else {
                    if (!ttlWrite.addParam(id, param_data.data())) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to add TTL param for SYNC_WRITE, ID: %d", id);
                    }
                    ttl_has_param = true;
                }
            }

            if (ttl_has_param) {
                int result = ttlWrite.txPacket();
                if (result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE TTL failed: %s", packet_handler_->getTxRxResult(result));
                }
            }

            if (rs_has_param) {
                int result = rsWrite.txPacket();
                if (result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE RS485 failed: %s", packet_handler_->getTxRxResult(result));
                }
            }

            response_data.push_back(dxl_error);
            publish_response(instr, response_data);
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
