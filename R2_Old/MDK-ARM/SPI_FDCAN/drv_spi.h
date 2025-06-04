/******************************************************************************
 *  Filename:    drv_spi.c
 *
 *  Description: MCP2518FD CAN FD controller driver implementation
 *
 *  Copyright (c) 2025 [UESTC_LIMITI] - All rights reserved
 *
 *  License:     GPL-3.0-or-later
 *
 *  Author:      [Chronologika]
 *  Contact:     [dianacoel@foxmail.com]
 *
 *  Version:     1.0.0
 *  Last Modified: 2025-04-30
 *
 *  SPDX-License-Identifier: GPL-3.0-or-later
 *****************************************************************************/
#ifndef _DRV_SPI_H
#define _DRV_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "spi.h"  // SPI_INIT_by_HAL
#include "gpio.h" // GPIO_INIT_by_HAL
#include "drv_canfdspi_api.h"

//========================================================//
#ifdef __cplusplus
extern "C"
{
#endif

#define DRV_CANFDSPI_INDEX_0 0
#define DRV_CANFDSPI_INDEX_1 1
#define BIT(_x_) ((uint32_t)(1U) << (_x_))
#define MTT_MAX_DATA_LEN 64
#define SPI_DEFAULT_BUFFER_LENGTH 96

/* Configuration Constants */

/* CAN Frame Format Flags */
#define CAN_FMT 0x80000000U /* Frame Format: 1=Extended, 0=Standard */
#define CAN_RTR 0x40000000U /* Remote Transmission Request */
#define CAN_ERR 0x20000000U /* Error Frame Indicator */

    /* Format of Canfd_frame */
    typedef struct
    {
        uint32_t can_id; /* FMT/RTR/ERR/ID */
        uint8_t d_len;   /* data length */
        uint8_t flags;   /* FD flags */
        uint8_t resv0;
        uint8_t resv1;
        uint8_t data[MTT_MAX_DATA_LEN];
        /* Any new structure entries should be placed below the comment */
        uint32_t tstamp;
    } canfd_frame;

    /* Function Prototypes */
    void DRV_SPI_Initialize(CAN_BITTIME_SETUP baud,
                            CAN_FIFO_CHANNEL txFifo,
                            CAN_FIFO_CHANNEL rxFifo);

    int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex,
                                uint8_t *SpiTxData,
                                uint8_t *SpiRxData,
                                uint16_t XferSize);

    int8_t DRV_SPI_ChipSelectAssert(uint8_t spiSlaveDeviceIndex,
                                    bool assert);

    int8_t MCP2518FD_SendData(CAN_FIFO_CHANNEL tx_fifo_channel,
                              bool is_extended,
                              bool use_fdcan,
                              bool enable_brs,
                              uint32_t can_id,
                              uint8_t *data,
                              uint8_t length);

    void MCP2518FD_ReceiveData(canfd_frame *RxFrame,
                               CAN_FIFO_CHANNEL rxFifo);

#ifdef __cplusplus
}
#endif
//========================================================//
#endif /* _DRV_SPI_H */
