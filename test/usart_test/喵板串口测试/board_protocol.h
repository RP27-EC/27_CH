#ifndef __BOARD_PROTOCOL_H
#define __BOARD_PROTOCOL_H

#include "stm32h7xx.h"
#include "stdbool.h"
#include "crc.h"
#include "string.h"
//板间通信测试
typedef struct {
    // -------------------- 1字节字段：uint8_t × 12个 --------------------
    uint8_t  u8_1;
    uint8_t  u8_2;
    uint8_t  u8_3;
    uint8_t  u8_4;
    uint8_t  u8_5;
    uint8_t  u8_6;
    uint8_t  u8_7;
    uint8_t  u8_8;
    uint8_t  u8_9;
    uint8_t  u8_10;
    uint8_t  u8_11;
    uint8_t  u8_12;

    // -------------------- 2字节字段：uint16_t × 8个 -------------------
    uint16_t u16_1;
    uint16_t u16_2;
    uint16_t u16_3;
    uint16_t u16_4;
    uint16_t u16_5;
    uint16_t u16_6;
    uint16_t u16_7;
    uint16_t u16_8;

    // -------------------- 4字节字段：uint32_t × 8个 -------------------
    uint32_t u32_1;
    uint32_t u32_2;
    uint32_t u32_3;
    uint32_t u32_4;
    uint32_t u32_5;
    uint32_t u32_6;
    uint32_t u32_7;
    uint32_t u32_8;
} TestStruct;


bool board_recieve(uint8_t *rxBuf);
void board_test_init(void);
void board_send(void);

#endif
