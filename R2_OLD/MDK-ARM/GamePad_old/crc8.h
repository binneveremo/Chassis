#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// 常见的 CRC-8 多项式和初始值
#define CRC8_INITIAL_VALUE 0x00
#define CRC8_POLYNOMIAL    0x07

/**
 * @brief 生成 CRC-8 校验码
 * @param data 数据指针
 * @param length 数据长度
 * @return 计算出的 CRC-8 值
 */
uint8_t crc8_generate(const uint8_t *data, size_t length);

/**
 * @brief 校验 CRC-8 值是否正确
 * @param data 数据指针
 * @param length 数据长度
 * @param received_crc 接收到的 CRC 值
 * @return true 校验正确，false 校验错误
 */
bool crc8_check(const uint8_t *data, size_t length, uint8_t received_crc);

#endif // CRC8_H
