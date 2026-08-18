#ifndef DART_VISION_SERIAL_PACKET_HPP
#define DART_VISION_SERIAL_PACKET_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace dart_vision::serial {

inline constexpr std::uint8_t kReceivePacketHeader = 0x5A;
inline constexpr std::uint8_t kSendPacketHeader = 0xA5;

/**
 * @brief 下位机发送给视觉端的固定长度协议包。
 *
 * 字节布局为 header(1) + target_id(1) + dart_id(1) + offset(4) + crc(2)，共 9 字节。
 * offset 是协议原始整数值，其角度单位和缩放倍率由上下位机共同约定。
 */
struct ReceivePacket {
    std::uint8_t header{kReceivePacketHeader};
    std::uint8_t target_id{};
    std::uint8_t dart_id{};
    std::int32_t offset{};
    std::uint16_t crc{};
} __attribute__((packed));

/**
 * @brief 视觉端发送给下位机的固定长度协议包。
 *
 * 字节布局为 header(1) + target_state(1) + stable(1) + yaw(4) + distance(4) + crc(2)，
 * 共 13 字节。yaw 和 distance 是协议原始整数值，其单位和缩放倍率由上下位机共同约定。
 */
struct SendPacket {
    std::uint8_t header{kSendPacketHeader};
    std::uint8_t target_state{};
    std::uint8_t stable{};
    std::int32_t yaw{};
    std::int32_t distance{};
    std::uint16_t crc{};
} __attribute__((packed));

static_assert(sizeof(ReceivePacket) == 9, "ReceivePacket protocol size mismatch");
static_assert(offsetof(ReceivePacket, target_id) == 1, "ReceivePacket target_id offset mismatch");
static_assert(offsetof(ReceivePacket, dart_id) == 2, "ReceivePacket dart_id offset mismatch");
static_assert(offsetof(ReceivePacket, offset) == 3, "ReceivePacket offset field mismatch");
static_assert(offsetof(ReceivePacket, crc) == 7, "ReceivePacket CRC offset mismatch");

static_assert(sizeof(SendPacket) == 13, "SendPacket protocol size mismatch");
static_assert(offsetof(SendPacket, target_state) == 1, "SendPacket target_state offset mismatch");
static_assert(offsetof(SendPacket, stable) == 2, "SendPacket stable offset mismatch");
static_assert(offsetof(SendPacket, yaw) == 3, "SendPacket yaw offset mismatch");
static_assert(offsetof(SendPacket, distance) == 7, "SendPacket distance offset mismatch");
static_assert(offsetof(SendPacket, crc) == 11, "SendPacket CRC offset mismatch");

/**
 * @brief 串口协议包类型。
 */
enum class PacketType {
    kReceive, ///< 下位机发送给视觉端的数据包。
    kSend,    ///< 视觉端发送给下位机的数据包。
    kUnknown  ///< 未知帧头，无法判断包类型。
};

/**
 * @brief 根据首字节判断协议包类型。
 *
 * @param header 待判断的帧头。
 * @return 与帧头对应的包类型；无法识别时返回 PacketType::kUnknown。
 */
[[nodiscard]] PacketType packetTypeFromHeader(std::uint8_t header) noexcept;

/**
 * @brief 根据帧头返回对应协议包的完整字节数。
 *
 * @param header 协议帧头。
 * @return ReceivePacket 或 SendPacket 的固定长度；未知帧头返回 0。
 */
[[nodiscard]] std::size_t packetSizeFromHeader(std::uint8_t header) noexcept;

/**
 * @brief 将发送包转换为可写入串口的连续字节，并计算末尾 CRC16。
 *
 * 函数会强制写入 kSendPacketHeader，并覆盖传入包原有的 crc 字段。参数按值传递，因此不会
 * 修改调用方持有的 SendPacket。
 *
 * @param packet 待编码的发送数据。
 * @return 长度固定为 sizeof(SendPacket) 的完整协议帧。
 */
[[nodiscard]] std::vector<std::uint8_t> encodeSendPacket(SendPacket packet);

/**
 * @brief 校验并解码下位机接收帧。
 *
 * 依次检查帧长度、帧头和 CRC16，全部通过后才复制到 ReceivePacket。该函数自身执行 CRC
 * 校验，因此即使调用方没有先经过 PacketParser，也不会接受损坏数据。
 *
 * @param frame 包含帧头、负载和 CRC16 的完整字节帧。
 * @return 校验成功时返回 ReceivePacket，否则返回 std::nullopt。
 */
[[nodiscard]] std::optional<ReceivePacket>
decodeReceivePacket(const std::vector<std::uint8_t>& frame);

} // namespace dart_vision::serial

#endif // DART_VISION_SERIAL_PACKET_HPP
