#include "crc8.h"

// 生成 CRC-8 校验码
uint8_t crc8_generate(const uint8_t *data, size_t length) {
    uint8_t crc = CRC8_INITIAL_VALUE;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; // 与当前字节异或

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) { // 检查最高位是否为 1
                crc = (crc << 1) ^ CRC8_POLYNOMIAL; // 左移后异或多项式
            } else {
                crc = crc << 1; // 仅左移
            }
        }
    }

    return crc;
}

// 校验 CRC-8 是否正确
bool crc8_check(const uint8_t *data, size_t length, uint8_t received_crc) {
    uint8_t computed_crc = crc8_generate(data, length);
    return (computed_crc == received_crc);
}
