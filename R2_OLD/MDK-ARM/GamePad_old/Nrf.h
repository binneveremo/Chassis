#pragma once
#include "spi.h"
#include "stdbool.h"


#define MATHBSP_TYPEBASED_FUNC_REFERENCE_LIST(reference, func)          \
    reference(double    ,   func##_double)                              \
    reference(float     ,   func##_float)                               \
    reference(uint64_t  ,   func##_uint64)                              \
    reference(int64_t   ,   func##_int64)                               \
    reference(uint32_t  ,   func##_uint32)                              \
    reference(int32_t   ,   func##_int32)                               \
    reference(uint16_t  ,   func##_uint16)                              \
    reference(int16_t   ,   func##_int16)                               \
    reference(uint8_t   ,   func##_uint8)                               \
    reference(int8_t    ,   func##_int8)
#define MATHBSP_TYPEBASED_INTEGER_FUNC_REFERENCE_LIST(reference, func)  \
    reference(uint64_t  ,   func##_uint64)                              \
    reference(int64_t   ,   func##_int64)                               \
    reference(uint32_t  ,   func##_uint32)                              \
    reference(int32_t   ,   func##_int32)                               \
    reference(uint16_t  ,   func##_uint16)                              \
    reference(int16_t   ,   func##_int16)                               \
    reference(uint8_t   ,   func##_uint8)                               \
    reference(int8_t    ,   func##_int8)

/*
*   归并排序：支持基本类型
*   type* MathBsp_MergeSort_type(type* source, type* buff, uint32_t len); 其中type为基本类型
*   source: 待排序数组
*   buff:   临时缓冲区
*   len:    数组长度
*   返回值: 排序后的数组(指向source)，其中source和buff都会得到排完后的数组
*/
#define MATHBSP_MERGESORT_REFERENCE(type, func)  type* func(type* source, type* buff, uint32_t len);
MATHBSP_TYPEBASED_FUNC_REFERENCE_LIST(MATHBSP_MERGESORT_REFERENCE, MathBsp_MergeSort)
/*
*   快速排序：支持基本类型
*   type* MathBsp_QuickSort_type(type* source, uint32_t len); 其中type为基本类型
*   source: 待排序数组
*   len:    数组长度
*   返回值: 排序后的数组(指向source)
*/
#define MATHBSP_QUICKSORT_REFERENCE(type, func)  type* func(type* source, uint32_t len);
MATHBSP_TYPEBASED_FUNC_REFERENCE_LIST(MATHBSP_QUICKSORT_REFERENCE, MathBsp_QuickSort)
/*
*   堆排序：支持基本类型
*   type* MathBsp_HeapSort_type(type* source, uint32_t len); 其中type为基本类型
*   source: 待排序数组
*   len:    数组长度
*   返回值: 排序后的数组(指向source)
*/
#define MATHBSP_HEAPSORT_REFERENCE(type, func)  type* func(type* source, uint32_t len);
MATHBSP_TYPEBASED_FUNC_REFERENCE_LIST(MATHBSP_HEAPSORT_REFERENCE, MathBsp_HeapSort)

//获取一个整数的二进制表示中最高位的1的位置
#define MATHBSP_GETHIGHESTBIT_REFERENCE(type, func) uint32_t func(type num);
MATHBSP_TYPEBASED_INTEGER_FUNC_REFERENCE_LIST(MATHBSP_GETHIGHESTBIT_REFERENCE, MathBsp_GetHighestBit)
//获取一个数的二进制表示中最低位的1的位置
#define MATHBSP_GETLOWESTBIT_REFERENCE(type, func) uint32_t func(type num);
MATHBSP_TYPEBASED_INTEGER_FUNC_REFERENCE_LIST(MATHBSP_GETLOWESTBIT_REFERENCE, MathBsp_GetLowestBit)
//将一个整数的第n位设置为1/0
#define MATHBSP_SETBIT_REFERENCE(type, func) type func(type num, uint32_t n, bool bitstate);
MATHBSP_TYPEBASED_INTEGER_FUNC_REFERENCE_LIST(MATHBSP_SETBIT_REFERENCE, MathBsp_SetBit)
//计算一个整数的二进制表示中1的个数
#define MATHBSP_COUNTBIT_REFERENCE(type, func) uint32_t func(type num);
MATHBSP_TYPEBASED_INTEGER_FUNC_REFERENCE_LIST(MATHBSP_COUNTBIT_REFERENCE, MathBsp_CountBit)
//获取一个二进制整数的第n位是1还是0
#define MATHBSP_GETBIT_REFERENCE(type, func) bool func(type num, uint32_t n);
MATHBSP_TYPEBASED_INTEGER_FUNC_REFERENCE_LIST(MATHBSP_GETBIT_REFERENCE, MathBsp_GetBit)





















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
