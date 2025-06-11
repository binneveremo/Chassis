#pragma once
#include "main.h"
#include "spi.h"

typedef struct 
{
    SPI_HandleTypeDef* hspi;
    struct 
    {
        GPIO_TypeDef* port;
        uint16_t pin;
    } ce, nss, irq, rst;
    struct 
    {
        //* Nrf可以监听6个地址，P0-P5，但是P0需要进行发送，所以只监听P1-P5
        //* 接收地址由最低字节地址p1-5和它们的高字节地址high_addr组成
        //* 也就是说，P1-5通道的监听地址除了最低字节地址可变外，高字节地址是固定的
        uint8_t p1;
        uint8_t p2;
        uint8_t p3;
        uint8_t p4;
        uint8_t p5;
        uint8_t high_addr[4];
    } address_receive;              // 接收地址
    uint8_t address_transmit[5];    // 发送地址
    uint8_t rf_channel;             // 射频通道
    struct              
    {
        uint8_t buf[32];
        uint8_t len;
    } rx_data[6];                   // 各个通道的接收数据

    enum Nrf_TxState_t
    {
        Nrf_Transmit_Ongoing,
        Nrf_Transmit_Success,
        Nrf_Transmit_Failed,
        Nrf_Transmit_Idle
    } tx_state;           //NRF发送状态
    enum Nrf_Mode_t
    {
        Nrf_Mode_Transmit,
        Nrf_Mode_Receive
    } mode;                 //NRF模式

    struct Nrf_SignalQuality_t
    {
        uint64_t check_buf[2];          //记录最近的64*2 = 128次的信号发送是否成功
        uint16_t check_index;           //当前检查的位置
        uint16_t check_success;         //成功的次数
        float quality;                  //信号质量
        enum Nrf_SignalQualityLevel_t
        {
            Nrf_SignalQualityLevel_Weak,
            Nrf_SignalQualityLevel_Normal,
            Nrf_SignalQualityLevel_Strong
        } level;                        //信号质量等级
    } signal_quality;       //信号质量

    void (*nrf_rx_callback)(uint8_t channel, uint8_t* data, uint8_t len); //接收回调函数
} Nrf_t;


void Nrf_Init(Nrf_t* nrf);
uint8_t _Nrf_CheckConnectivity(Nrf_t* nrf);
void Nrf_EXTI_Callback(Nrf_t* nrf,uint16_t gpio_pin);
void Nrf_Transmit(Nrf_t* nrf, uint8_t* data, uint8_t len);
void Nrf_SetTransmitAddress(Nrf_t* nrf, uint8_t* address);
void Nrf_TurnOff(Nrf_t* nrf);
void Nrf_TurnOn(Nrf_t* nrf);
void Nrf_Reset(Nrf_t* nrf);
