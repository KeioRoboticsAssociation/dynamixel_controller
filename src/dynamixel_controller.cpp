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
#include "dynamixel_controller/msg/dynamixel_command.hpp"
#include "dynamixel_controller/msg/dynamixel_response.hpp"

#include "dynamixel_controller/dynamixel_controller.hpp"
#include "dynamixel_controller/msg/dynamixel_controller.hpp"

// 接続情報のマクロ（必要に応じて調整）
#define BAUDRATE 57600
#define DEVICE_NAME "/dev/ttyUSB0"

// メッセージ定義のショートカット
#define MSG dynamixel_controller::msg::DynamixelController


DynamixelController::DynamixelController() : Node("dynamixel_controller_node"), port_handler_(nullptr), packet_handler_(nullptr) {
    RCLCPP_INFO(this->get_logger(), "DynamixelController node started.");

    // ポートハンドラ、パケットハンドラの初期化
    port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    if (!port_handler_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create port handler for: %s", DEVICE_NAME);
        rclcpp::shutdown();
        return;
    }

    if (!port_handler_->openPort()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", DEVICE_NAME);
        rclcpp::shutdown();
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Port opened: %s", DEVICE_NAME);
    }

    if (!port_handler_->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate: %d", BAUDRATE);
        port_handler_->closePort();
        rclcpp::shutdown();
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Baudrate set: %d", BAUDRATE);
    }

    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);
    if (!packet_handler_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create packet handler for protocol version: %.1f", protocol_version_);
        if (port_handler_) {
            port_handler_->closePort();
        }
        rclcpp::shutdown();
        return;
    }

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
    instruction_subscriber_ = this->create_subscription<dynamixel_controller::msg::DynamixelCommand>(
        "dynamixel_tx", 100,
        std::bind(&DynamixelController::instruction_callback, this, std::placeholders::_1));

    // ROS2 パブリッシャーの作成 (受信応答を publish する)
    response_publisher_ = this->create_publisher<dynamixel_controller::msg::DynamixelResponse>("dynamixel_rx", 10);
}

DynamixelController::~DynamixelController() {
    // ポートを閉じ、リソース解放
    if (port_handler_) {
        port_handler_->closePort();
        delete port_handler_;
        port_handler_ = nullptr;
    }
    // packet_handler_ はシングルトンのため、削除不要
    packet_handler_ = nullptr;
}

void DynamixelController::publish_response(uint8_t instruction_code, const std::vector<uint8_t> & ids, const std::vector<uint8_t> & errors, const std::vector<uint8_t> & data) {
    dynamixel_controller::msg::DynamixelResponse msg;

    msg.command = instruction_code;
    msg.ids = ids;
    msg.error = errors;
    msg.data = data;

    response_publisher_->publish(msg);
}

void DynamixelController::instruction_callback(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg) {
    switch (msg->command) {
        case MSG::READ_DATA: {
            handle_read_command(msg);
            break;
        }
        case MSG::WRITE_DATA: {
            handle_write_command(msg);
            break;
        }
        case MSG::SYNC_READ: {
            handle_sync_read_command(msg);
            break;
        }
        case MSG::SYNC_WRITE: {
            handle_sync_write_command(msg);
            break;
        }
        case MSG::PING: {
            handle_ping_command(msg);
            break;
        }
        default:
            RCLCPP_WARN(this->get_logger(), "Received unsupported command: %d", msg->command);
            break;
    }
}

void DynamixelController::handle_read_command(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg) {
    if (!msg || msg->ids.empty()) {
        RCLCPP_ERROR(this->get_logger(), "READ command requires at least one ID");
        return;
    }

    if (!port_handler_ || !packet_handler_) {
        RCLCPP_ERROR(this->get_logger(), "Port handler or packet handler not initialized");
        return;
    }

    uint8_t dxl_id = msg->ids[0];
    uint16_t read_address = msg->address;
    uint16_t read_length = msg->length;
    uint8_t dxl_error = 0;
    int comm_result = COMM_TX_FAIL;
    std::vector<uint8_t> response_data;

    if (read_length == 1) {
        uint8_t data = 0;
        comm_result = packet_handler_->read1ByteTxRx(port_handler_, dxl_id, read_address, &data, &dxl_error);
        if (comm_result == COMM_SUCCESS) {
            response_data.push_back(data);
        }
    } else if (read_length == 2) {
        uint16_t data = 0;
        comm_result = packet_handler_->read2ByteTxRx(port_handler_, dxl_id, read_address, &data, &dxl_error);
        if (comm_result == COMM_SUCCESS) {
            response_data.push_back(static_cast<uint8_t>(data & 0xFF));
            response_data.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
        }
    } else if (read_length == 4) {
        uint32_t data = 0;
        comm_result = packet_handler_->read4ByteTxRx(port_handler_, dxl_id, read_address, &data, &dxl_error);
        if (comm_result == COMM_SUCCESS) {
            response_data.push_back(static_cast<uint8_t>(data & 0xFF));
            response_data.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
            response_data.push_back(static_cast<uint8_t>((data >> 16) & 0xFF));
            response_data.push_back(static_cast<uint8_t>((data >> 24) & 0xFF));
        }
    }

    std::vector<uint8_t> ids = {dxl_id};
    std::vector<uint8_t> errors = {dxl_error};

    if (comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Read failed for ID %d: %s", dxl_id, packet_handler_->getTxRxResult(comm_result));
        errors[0] = 255; // Communication error
    } else {
        // Log successful read communication
        std::string data_str = "";
        for (size_t i = 0; i < response_data.size(); i++) {
            if (i > 0) data_str += " ";
            data_str += std::to_string(response_data[i]);
        }
        RCLCPP_INFO(this->get_logger(), "READ_DATA success - ID: %d, Address: %d, Length: %d, Data: [%s], Error: %d",
                    dxl_id, read_address, read_length, data_str.c_str(), dxl_error);
    }

    publish_response(MSG::READ_DATA, ids, errors, response_data);
}

void DynamixelController::handle_write_command(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg) {
    if (!msg || msg->ids.empty() || msg->data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "WRITE command requires ID and data");
        return;
    }

    if (!port_handler_ || !packet_handler_) {
        RCLCPP_ERROR(this->get_logger(), "Port handler or packet handler not initialized");
        return;
    }

    uint8_t dxl_id = msg->ids[0];
    uint16_t write_address = msg->address;
    uint16_t data_length = msg->length;
    uint8_t dxl_error = 0;
    int comm_result = COMM_TX_FAIL;

    if (data_length == 1) {
        if (msg->data.size() < 1) {
            RCLCPP_ERROR(this->get_logger(), "Insufficient data for 1-byte write. Expected: 1, Got: %zu", msg->data.size());
            return;
        }
        uint8_t value = msg->data[0];
        comm_result = packet_handler_->write1ByteTxRx(port_handler_, dxl_id, write_address, value, &dxl_error);
    } else if (data_length == 2) {
        if (msg->data.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Insufficient data for 2-byte write. Expected: 2, Got: %zu", msg->data.size());
            return;
        }
        uint16_t value = msg->data[0] | (msg->data[1] << 8);
        comm_result = packet_handler_->write2ByteTxRx(port_handler_, dxl_id, write_address, value, &dxl_error);
    } else if (data_length == 4) {
        if (msg->data.size() < 4) {
            RCLCPP_ERROR(this->get_logger(), "Insufficient data for 4-byte write. Expected: 4, Got: %zu", msg->data.size());
            return;
        }
        uint32_t value = msg->data[0] | (msg->data[1] << 8) | (msg->data[2] << 16) | (msg->data[3] << 24);
        comm_result = packet_handler_->write4ByteTxRx(port_handler_, dxl_id, write_address, value, &dxl_error);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported data length: %d. Supported: 1, 2, 4 bytes", data_length);
        return;
    }

    std::vector<uint8_t> ids = {dxl_id};
    std::vector<uint8_t> errors = {dxl_error};
    std::vector<uint8_t> response_data;

    if (comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Write failed for ID %d: %s", dxl_id, packet_handler_->getTxRxResult(comm_result));
        errors[0] = 255; // Communication error
    } else {
        // Log successful write communication
        std::string data_str = "";
        for (size_t i = 0; i < data_length && i < msg->data.size(); i++) {
            if (i > 0) data_str += " ";
            data_str += std::to_string(msg->data[i]);
        }
        RCLCPP_INFO(this->get_logger(), "WRITE_DATA success - ID: %d, Address: %d, Length: %d, Data: [%s], Error: %d",
                    dxl_id, write_address, data_length, data_str.c_str(), dxl_error);
    }

    publish_response(MSG::WRITE_DATA, ids, errors, response_data);
}

void DynamixelController::handle_sync_read_command(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg) {
    if (!msg || msg->ids.empty()) {
        RCLCPP_ERROR(this->get_logger(), "SYNC_READ command requires at least one ID");
        return;
    }

    if (!port_handler_ || !packet_handler_) {
        RCLCPP_ERROR(this->get_logger(), "Port handler or packet handler not initialized");
        return;
    }

    uint16_t start_address = msg->address;
    uint16_t data_length = msg->length;
    std::vector<uint8_t> id_list = msg->ids;

    std::vector<uint8_t> ttl_targets;
    std::vector<uint8_t> rs_targets;
    for (auto id : id_list) {
        if (rs485_ids_.count(id)) {
            rs_targets.push_back(id);
        } else {
            ttl_targets.push_back(id);
        }
    }

    std::vector<uint8_t> response_ids;
    std::vector<uint8_t> response_errors;
    std::vector<uint8_t> response_data;

    // Handle TTL devices
    if (!ttl_targets.empty()) {
        dynamixel::GroupSyncRead ttlRead(port_handler_, packet_handler_, start_address, data_length);
        for (auto id : ttl_targets) {
            ttlRead.addParam(id);
        }
        int comm_result = ttlRead.txRxPacket();
        if (comm_result == COMM_SUCCESS) {
            for (auto id : ttl_targets) {
                if (ttlRead.isAvailable(id, start_address, data_length)) {
                    uint32_t data = ttlRead.getData(id, start_address, data_length);
                    response_ids.push_back(id);
                    response_errors.push_back(0); // No error

                    std::string data_str = "";
                    for (uint8_t i = 0; i < data_length; i++) {
                        uint8_t byte_data = static_cast<uint8_t>((data >> (i*8)) & 0xFF);
                        response_data.push_back(byte_data);
                        if (i > 0) data_str += " ";
                        data_str += std::to_string(byte_data);
                    }
                    RCLCPP_INFO(this->get_logger(), "SYNC_READ success (TTL) - ID: %d, Address: %d, Length: %d, Data: [%s]",
                                id, start_address, data_length, data_str.c_str());
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "SYNC_READ failed for TTL devices: %s", packet_handler_->getTxRxResult(comm_result));
        }
    }

    // Handle RS485 devices
    if (!rs_targets.empty()) {
        dynamixel::GroupSyncRead rsRead(port_handler_, packet_handler_, start_address, data_length);
        for (auto id : rs_targets) {
            rsRead.addParam(id);
        }
        int comm_result = rsRead.txRxPacket();
        if (comm_result == COMM_SUCCESS) {
            for (auto id : rs_targets) {
                if (rsRead.isAvailable(id, start_address, data_length)) {
                    uint32_t data = rsRead.getData(id, start_address, data_length);
                    response_ids.push_back(id);
                    response_errors.push_back(0); // No error

                    std::string data_str = "";
                    for (uint8_t i = 0; i < data_length; i++) {
                        uint8_t byte_data = static_cast<uint8_t>((data >> (i*8)) & 0xFF);
                        response_data.push_back(byte_data);
                        if (i > 0) data_str += " ";
                        data_str += std::to_string(byte_data);
                    }
                    RCLCPP_INFO(this->get_logger(), "SYNC_READ success (RS485) - ID: %d, Address: %d, Length: %d, Data: [%s]",
                                id, start_address, data_length, data_str.c_str());
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "SYNC_READ failed for RS485 devices: %s", packet_handler_->getTxRxResult(comm_result));
        }
    }

    publish_response(MSG::SYNC_READ, response_ids, response_errors, response_data);
}

void DynamixelController::handle_sync_write_command(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg) {
    if (!msg || msg->ids.empty() || msg->data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE command requires IDs and data");
        return;
    }

    if (!port_handler_ || !packet_handler_) {
        RCLCPP_ERROR(this->get_logger(), "Port handler or packet handler not initialized");
        return;
    }

    uint16_t start_address = msg->address;
    uint16_t data_length = msg->length;

    size_t expected_data_size = msg->ids.size() * data_length;
    if (msg->data.size() != expected_data_size) {
        RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE: data size mismatch. Expected %zu bytes, got %zu",
                     expected_data_size, msg->data.size());
        return;
    }

    if (data_length == 0 || data_length > 4) {
        RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE: invalid data length %d. Supported: 1-4 bytes", data_length);
        return;
    }

    dynamixel::GroupSyncWrite ttlWrite(port_handler_, packet_handler_, start_address, data_length);
    dynamixel::GroupSyncWrite rsWrite(port_handler_, packet_handler_, start_address, data_length);
    bool ttl_has_param = false;
    bool rs_has_param = false;

    for (size_t i = 0; i < msg->ids.size(); i++) {
        uint8_t id = msg->ids[i];
        std::vector<uint8_t> param_data;

        for (uint16_t j = 0; j < data_length; j++) {
            size_t data_index = i * data_length + j;
            if (data_index >= msg->data.size()) {
                RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE: data index out of bounds. Index: %zu, Size: %zu",
                             data_index, msg->data.size());
                return;
            }
            param_data.push_back(msg->data[data_index]);
        }

        if (rs485_ids_.count(id)) {
            rsWrite.addParam(id, param_data.data());
            rs_has_param = true;
        } else {
            ttlWrite.addParam(id, param_data.data());
            ttl_has_param = true;
        }
    }

    if (ttl_has_param) {
        int comm_result = ttlWrite.txPacket();
        if (comm_result == COMM_SUCCESS) {
            std::string id_str = "";
            for (size_t i = 0; i < msg->ids.size(); i++) {
                if (ttl_ids_.count(msg->ids[i]) || !rs485_ids_.count(msg->ids[i])) {
                    if (!id_str.empty()) id_str += ", ";
                    id_str += std::to_string(msg->ids[i]);
                }
            }
            RCLCPP_INFO(this->get_logger(), "SYNC_WRITE success (TTL) - IDs: [%s], Address: %d, Length: %d",
                        id_str.c_str(), start_address, data_length);
        } else {
            RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE failed for TTL devices: %s", packet_handler_->getTxRxResult(comm_result));
        }
    }

    if (rs_has_param) {
        int comm_result = rsWrite.txPacket();
        if (comm_result == COMM_SUCCESS) {
            std::string id_str = "";
            for (size_t i = 0; i < msg->ids.size(); i++) {
                if (rs485_ids_.count(msg->ids[i])) {
                    if (!id_str.empty()) id_str += ", ";
                    id_str += std::to_string(msg->ids[i]);
                }
            }
            RCLCPP_INFO(this->get_logger(), "SYNC_WRITE success (RS485) - IDs: [%s], Address: %d, Length: %d",
                        id_str.c_str(), start_address, data_length);
        } else {
            RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE failed for RS485 devices: %s", packet_handler_->getTxRxResult(comm_result));
        }
    }

    std::vector<uint8_t> response_ids = msg->ids;
    std::vector<uint8_t> response_errors(msg->ids.size(), 0); // All success
    std::vector<uint8_t> response_data;

    publish_response(MSG::SYNC_WRITE, response_ids, response_errors, response_data);
}

void DynamixelController::handle_ping_command(const dynamixel_controller::msg::DynamixelCommand::SharedPtr msg) {
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Invalid message pointer");
        return;
    }

    if (!port_handler_ || !packet_handler_) {
        RCLCPP_ERROR(this->get_logger(), "Port handler or packet handler not initialized");
        return;
    }

    std::vector<uint8_t> response_ids;
    std::vector<uint8_t> response_errors;
    std::vector<uint8_t> response_data;

    for (auto id : msg->ids) {
        uint8_t dxl_error = 0;
        int comm_result = packet_handler_->ping(port_handler_, id, &dxl_error);

        response_ids.push_back(id);
        if (comm_result == COMM_SUCCESS) {
            response_errors.push_back(dxl_error);
            RCLCPP_INFO(this->get_logger(), "PING success - ID: %d, Error: %d", id, dxl_error);
        } else {
            response_errors.push_back(255); // Communication error
            RCLCPP_WARN(this->get_logger(), "Ping failed for ID %d: %s", id, packet_handler_->getTxRxResult(comm_result));
        }
    }

    publish_response(MSG::PING, response_ids, response_errors, response_data);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}