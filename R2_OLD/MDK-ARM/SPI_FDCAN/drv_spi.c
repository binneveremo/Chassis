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
#include "drv_spi.h"

/**
 * @brief  Control SPI chip select signal
 * @param  spiSlaveDeviceIndex: Multiplexed SPI device selection (0-based index)
 * @param  assert: Chip select assertion state (true=active, false=inactive)
 * @retval Execution status: 0 for success, -1 for invalid device index
 */
int8_t DRV_SPI_ChipSelectAssert(uint8_t spiSlaveDeviceIndex, bool assert)
{
    int8_t error = 0;

    // Select Chip Select
    switch (spiSlaveDeviceIndex)
    {
    case DRV_CANFDSPI_INDEX_0:
        if (assert)
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
        break;
    default:
        error = -1;
        break;
    }
    return error;
}

/**
 * @brief  Perform SPI data transfer with full-duplex communication
 * @param  spiSlaveDeviceIndex: Multiplexed SPI device selection (0-based index)
 * @param  SpiTxData: Transmit data buffer pointer
 * @param  SpiRxData: Receive data buffer pointer
 * @param  spiTransferSize: Number of bytes to transfer
 * @retval Chip select status: 0 for success, -1 for invalid device index
 */
int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
    int8_t error = 0;
    // Assert CS
    error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, true);
    if (error != 0)
        return error;

    switch (spiSlaveDeviceIndex)
    {
    case DRV_CANFDSPI_INDEX_0:
        HAL_SPI_TransmitReceive(&hspi5, SpiTxData, SpiRxData, spiTransferSize, 10);
        break;
    default:
        break;
    }
    // De-assert CS
    error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, false);

    return error;
}

/**
 * @brief  Initialize MCP2518FD controller and SPI interface
 * @param  baud: Bit timing configuration structure
 * @param  txFifo: Transmit FIFO channel selection
 * @param  rxFifo: Receive FIFO channel selection
 * @retval None
 */
void DRV_SPI_Initialize(CAN_BITTIME_SETUP baud, CAN_FIFO_CHANNEL txFifo, CAN_FIFO_CHANNEL rxFifo)
{
    // Reset MCP2518FD and enter configuration mode
    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);

    // Enable ECC and initialize RAM
    DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);
    DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xFF);

    // Configure clock output
    CAN_OSC_CTRL oscCtrl;
    DRV_CANFDSPI_OscillatorControlObjectReset(&oscCtrl);
    oscCtrl.ClkOutDivide = OSC_CLKO_DIV10;
    DRV_CANFDSPI_OscillatorControlSet(DRV_CANFDSPI_INDEX_0, oscCtrl);

    // Configure INT0 and INT1 as interrupt outputs
    DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);

    // Configure the CAN controller
    CAN_CONFIG config;
    DRV_CANFDSPI_ConfigureObjectReset(&config);
    config.IsoCrcEnable = 1;
    config.StoreInTEF = 0;
    config.TXQEnable = 0;
    config.BitRateSwitchDisable = 0;
    DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);

    // Set CAN bit timing
    DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, baud, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);

    // Configure transmit FIFO
    CAN_TX_FIFO_CONFIG txConfig;
    DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
    txConfig.FifoSize = 7;
    txConfig.PayLoadSize = CAN_PLSIZE_64;
    txConfig.TxPriority = 0;
    DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, txFifo, &txConfig);
    DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_0, txFifo, CAN_TX_FIFO_EMPTY_EVENT);

    // Configure receive FIFO
    CAN_RX_FIFO_CONFIG rxConfig;
    DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
    rxConfig.FifoSize = 15;
    rxConfig.PayLoadSize = CAN_PLSIZE_64;
    rxConfig.RxTimeStampEnable = 0;
    DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, rxFifo, &rxConfig);
    DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, rxFifo, CAN_RX_FIFO_NOT_EMPTY_EVENT);

    // Configure RX filter to receive all standard IDs
    CAN_FILTEROBJ_ID fObj = {0};
    CAN_MASKOBJ_ID mObj = {0};
    fObj.SID = 0x000;
    fObj.EXIDE = 0;
    mObj.MSID = 0x000;
    mObj.MIDE = 0;
    DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &fObj);
    DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &mObj);
    DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, rxFifo, true);
    DRV_CANFDSPI_FilterEnable(DRV_CANFDSPI_INDEX_0, CAN_FILTER0);

    // Switch to normal operation mode
    DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);

    // Enable interrupts
    DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0,
                                   CAN_RX_EVENT |
                                       CAN_TX_EVENT |
                                       CAN_BUS_ERROR_EVENT |
                                       CAN_RX_OVERFLOW_EVENT |
                                       CAN_SYSTEM_ERROR_EVENT);
}

/**
 * @brief  Transmit CAN FD frame via MCP2518FD controller using specified FIFO channel
 * @param  tx_fifo_channel: Transmit FIFO channel to use (e.g., CAN_FIFO_CH1)
 * @param  is_extended: Indicates extended frame format (29-bit ID)
 * @param  use_fdcan: Enable CAN FD frame format
 * @param  enable_brs: Enable bit rate switching
 * @param  can_id: CAN identifier (11/29-bit depending on format)
 * @param  data: Pointer to payload data buffer
 * @param  length: Data payload length (0-64 bytes)
 * @retval Execution status: 0 for success, negative values for errors
 */
int8_t MCP2518FD_SendData(CAN_FIFO_CHANNEL tx_fifo_channel,
                          bool is_extended,
                          bool use_fdcan,
                          bool enable_brs,
                          uint32_t can_id,
                          uint8_t *data,
                          uint8_t length)
{
    if (length > 64)
        return -1; // Invalid data length

    CAN_TX_MSGOBJ txObj;
    memset(&txObj, 0, sizeof(txObj));

    if (is_extended)
    {
        txObj.bF.ctrl.IDE = 1;
        txObj.bF.id.SID = (can_id >> 18) & 0x7FF;
        txObj.bF.id.EID = can_id & 0x3FFFF;
    }
    else
    {
        txObj.bF.ctrl.IDE = 0;
        txObj.bF.id.SID = can_id & 0x7FF;
    }

    txObj.bF.ctrl.FDF = use_fdcan ? 1 : 0;
    txObj.bF.ctrl.BRS = enable_brs ? 1 : 0;
    txObj.bF.ctrl.DLC = DRV_CANFDSPI_DataBytesToDlc(length);

    return DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0,
                                            tx_fifo_channel,
                                            &txObj,
                                            data,
                                            length,
                                            true);
}

/**
 * @brief  Receive CAN FD frame from specified FIFO buffer
 * @param  RxFrame: Pointer to canfd_frame structure for received data
 * @param  rxFifo: FIFO channel selector (CAN_FIFO_CH0-CAN_FIFO_CH15)
 * @retval None
 */
void MCP2518FD_ReceiveData(canfd_frame *RxFrame, CAN_FIFO_CHANNEL rxFifo)
{
    // Clear the receive buffer
    memset(RxFrame, 0, sizeof(canfd_frame));

    uint32_t rxif;
    CAN_RXCODE rxCode;
    CAN_RX_MSGOBJ rxObj;

    // Get receive interrupt flags and receive code
    DRV_CANFDSPI_ReceiveEventGet(DRV_CANFDSPI_INDEX_0, &rxif);
    DRV_CANFDSPI_ModuleEventRxCodeGet(DRV_CANFDSPI_INDEX_0, &rxCode);

    // Check if the specified FIFO has data
    if ((rxif & BIT(rxFifo)) || (rxCode == (CAN_RXCODE)rxFifo))
    {
        // Attempt to read the received frame from the FIFO
        if (DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_0,
                                           rxFifo,
                                           &rxObj,
                                           RxFrame->data,
                                           MTT_MAX_DATA_LEN) == 0)
        {
            // Determine whether the frame is standard or extended
            if (rxObj.bF.ctrl.IDE)
            {
                RxFrame->can_id = (rxObj.bF.id.SID << 18) | rxObj.bF.id.EID;
            }
            else
            {
                RxFrame->can_id = rxObj.bF.id.SID;
            }

            // Determine actual data length
            RxFrame->d_len = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)rxObj.bF.ctrl.DLC);
        }
    }
}
