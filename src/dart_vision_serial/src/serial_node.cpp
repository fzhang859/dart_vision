#include "dart_vision_serial/serial_node.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "dart_vision_serial/packet.hpp"

namespace dart_vision::serial {

SerialNode::SerialNode(const rclcpp::NodeOptions& options) : Node("serial_node", options) {
    declareParameters();
    serial_config_ = readSerialConfig();

    const std::int64_t reconnect_interval_ms = get_parameter("reconnect_interval_ms").as_int();
    if (reconnect_interval_ms <= 0 || reconnect_interval_ms > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("reconnect_interval_ms is outside the valid range");
    }
    reconnect_interval_ = std::chrono::milliseconds(reconnect_interval_ms);

    yaw_scale_ = get_parameter("yaw_scale").as_double();
    distance_scale_ = get_parameter("distance_scale").as_double();
    offset_scale_ = get_parameter("offset_scale").as_double();
    if (!std::isfinite(yaw_scale_) || yaw_scale_ <= 0.0 || !std::isfinite(distance_scale_) ||
        distance_scale_ <= 0.0 || !std::isfinite(offset_scale_) || offset_scale_ <= 0.0) {
        throw std::invalid_argument("Protocol scales must be finite and greater than zero");
    }

    send_topic_ = get_parameter("send_topic").as_string();
    receive_topic_ = get_parameter("receive_topic").as_string();
    if (send_topic_.empty() || receive_topic_.empty()) {
        throw std::invalid_argument("Serial topic names must not be empty");
    }

    serial_port_ = std::make_unique<SerialPort>(serial_config_);
    receive_publisher_ = create_publisher<dart_vision_interfaces::msg::ControllerState>(
        receive_topic_, rclcpp::SensorDataQoS());
    send_subscription_ = create_subscription<dart_vision_interfaces::msg::AimCommand>(
        send_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&SerialNode::sendCallback, this, std::placeholders::_1));

    running_.store(true);
    receive_thread_ = std::thread(&SerialNode::receiveLoop, this);

    RCLCPP_INFO(get_logger(),
                "Serial node started: device=%s baud=%u send_topic=%s receive_topic=%s",
                serial_config_.device.c_str(),
                serial_config_.baud_rate,
                send_topic_.c_str(),
                receive_topic_.c_str());
}

SerialNode::~SerialNode() {
    running_.store(false);
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    closePort();
}

void SerialNode::declareParameters() {
    declare_parameter<std::string>("device", "/dev/ttyACM0");
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<int>("read_timeout_ms", 100);
    declare_parameter<int>("reconnect_interval_ms", 1000);
    declare_parameter<double>("yaw_scale", 1000.0);
    declare_parameter<double>("distance_scale", 1000.0);
    declare_parameter<double>("offset_scale", 1000.0);
    declare_parameter<std::string>("send_topic", "aim_command");
    declare_parameter<std::string>("receive_topic", "controller_state");
}

SerialConfig SerialNode::readSerialConfig() const {
    const std::int64_t baud_rate = get_parameter("baud_rate").as_int();
    const std::int64_t read_timeout_ms = get_parameter("read_timeout_ms").as_int();

    if (baud_rate <= 0 ||
        baud_rate > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())) {
        throw std::invalid_argument("baud_rate is outside uint32 range");
    }
    if (read_timeout_ms < 0 || read_timeout_ms > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("read_timeout_ms is outside int range");
    }

    SerialConfig config;
    config.device = get_parameter("device").as_string();
    config.baud_rate = static_cast<std::uint32_t>(baud_rate);
    config.read_timeout_ms = static_cast<int>(read_timeout_ms);
    return config;
}

void SerialNode::receiveLoop() {
    std::array<std::uint8_t, 256> read_buffer{};

    while (running_.load() && rclcpp::ok()) {
        if (!connected_.load() || reconnect_requested_.exchange(false)) {
            closePort();
            packet_parser_.reset();

            if (!tryOpenPort()) {
                std::this_thread::sleep_for(reconnect_interval_);
                continue;
            }
        }

        try {
            const std::size_t bytes_read =
                serial_port_->read(read_buffer.data(), read_buffer.size());
            if (bytes_read == 0) {
                continue;
            }

            packet_parser_.append(read_buffer.data(), bytes_read);
            processBufferedFrames();
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "Serial receive failed: %s", error.what());
            requestReconnect();
        }
    }
}

void SerialNode::processBufferedFrames() {
    while (running_.load() && rclcpp::ok()) {
        ParseResult result = packet_parser_.nextFrame();

        switch (result.status) {
            case ParseStatus::kNeedMoreData:
                return;
            case ParseStatus::kFrameReady:
                processFrame(result.frame);
                break;
            case ParseStatus::kCRCError:
                RCLCPP_WARN_THROTTLE(get_logger(),
                                     *get_clock(),
                                     2000,
                                     "Received a %zu-byte frame with invalid CRC16",
                                     result.frame.size());
                break;
            case ParseStatus::kUnknownHeader:
                RCLCPP_WARN_THROTTLE(get_logger(),
                                     *get_clock(),
                                     2000,
                                     "Discarded %zu byte(s) before a valid receive header",
                                     result.frame.size());
                break;
        }
    }
}

void SerialNode::processFrame(const std::vector<std::uint8_t>& frame) {
    const auto packet = decodeReceivePacket(frame);
    if (!packet.has_value()) {
        RCLCPP_WARN(get_logger(), "A parser-approved receive frame failed packet decoding");
        return;
    }

    dart_vision_interfaces::msg::ControllerState message;
    message.header.stamp = now();
    message.header.frame_id = "serial";
    message.target_id = packet->target_id;
    message.dart_id = packet->dart_id;
    message.offset_raw = packet->offset;
    message.offset_rad = static_cast<double>(packet->offset) / offset_scale_;
    receive_publisher_->publish(message);
}

void SerialNode::sendCallback(
    const dart_vision_interfaces::msg::AimCommand::ConstSharedPtr& message) {
    if (!connected_.load()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Dropping aim command: serial port is disconnected");
        return;
    }

    try {
        SendPacket packet;
        packet.target_state = message->target_state;
        packet.stable = message->stable ? 1U : 0U;
        packet.yaw = scaleToProtocol(message->yaw_rad, yaw_scale_);
        packet.distance = scaleToProtocol(message->distance_m, distance_scale_);

        const auto frame = encodeSendPacket(packet);
        std::lock_guard<std::mutex> lock(port_lifecycle_mutex_);
        if (!connected_.load() || !serial_port_->isOpen()) {
            return;
        }
        serial_port_->write(frame.data(), frame.size());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Serial send failed: %s", error.what());
        requestReconnect();
    }
}

bool SerialNode::tryOpenPort() {
    try {
        std::lock_guard<std::mutex> lock(port_lifecycle_mutex_);
        serial_port_->open();
        connected_.store(true);
        RCLCPP_INFO(get_logger(), "Opened serial device %s", serial_config_.device.c_str());
        return true;
    } catch (const std::exception& error) {
        connected_.store(false);
        RCLCPP_WARN(get_logger(),
                    "Unable to open serial device %s: %s",
                    serial_config_.device.c_str(),
                    error.what());
        return false;
    }
}

void SerialNode::closePort() noexcept {
    std::lock_guard<std::mutex> lock(port_lifecycle_mutex_);
    connected_.store(false);
    if (serial_port_) {
        serial_port_->close();
    }
}

void SerialNode::requestReconnect() noexcept {
    connected_.store(false);
    reconnect_requested_.store(true);
}

std::int32_t SerialNode::scaleToProtocol(double value, double scale) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument("Protocol value must be finite");
    }

    const double scaled = value * scale;
    if (!std::isfinite(scaled) || scaled < std::numeric_limits<std::int32_t>::min() ||
        scaled > std::numeric_limits<std::int32_t>::max()) {
        throw std::out_of_range("Scaled protocol value is outside int32 range");
    }

    return static_cast<std::int32_t>(std::llround(scaled));
}

} // namespace dart_vision::serial
