#include "SPI_FDCAN.h"
#include "Can_Bsp.h"
#include "fdcan.h"
#include "string.h"
int FDCAN_Send(FDCAN_HandleTypeDef *hfdcan,int head, char * idtype,unsigned char * send, char * cantype, int length, char * brs){
	if(hfdcan == &hfdcan4) goto SPI_FDCAN;
	FDCAN_TxHeaderTypeDef fdcan_TxHeader;
	fdcan_TxHeader.Identifier=head;
	//设置帧头模式
	if(strcmp(idtype,"STD") == 0)
	  fdcan_TxHeader.IdType=FDCAN_STANDARD_ID;   
  else if(strcmp(idtype,"EXT") == 0)
		fdcan_TxHeader.IdType=FDCAN_EXTENDED_ID; 
	fdcan_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	
	//设置长度
	if(length <= 8)
  	fdcan_TxHeader.DataLength = length; 
	else if(length == 12)
    fdcan_TxHeader.DataLength = FDCAN_DLC_BYTES_12;	
	else if(length == 16)
    fdcan_TxHeader.DataLength = FDCAN_DLC_BYTES_16;		
  else if(length == 20)
    fdcan_TxHeader.DataLength = FDCAN_DLC_BYTES_20;		
	else if(length == 24)
    fdcan_TxHeader.DataLength = FDCAN_DLC_BYTES_24;		
	else if(length == 32)
    fdcan_TxHeader.DataLength = FDCAN_DLC_BYTES_32;	
	fdcan_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  
	
	//设置可变波特率
  if(strcmp(brs,"ON") == 0)	
	  fdcan_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  else if(strcmp(brs,"OFF") == 0)	
	  fdcan_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	
	//设置CAN的模式
	if(strcmp(cantype,"CLASSIC") == 0)	
	  fdcan_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;   
  else if(strcmp(cantype,"FD") == 0)
    fdcan_TxHeader.FDFormat = FDCAN_FD_CAN;  		
	fdcan_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;     
	fdcan_TxHeader.MessageMarker=0;                           
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,&fdcan_TxHeader,send)==HAL_OK) return 1;
	return 2;	
	
SPI_FDCAN:
	MCP2518FD_SendData(CAN_FIFO_CH1, ((strcmp(idtype,"EXT"))?false:true),((strcmp(cantype,"FD"))?false:true), ((strcmp(brs,"ON"))?false:true), head, send, length);
	return 1;
}
unsigned char FDCAN_SendData(FDCAN_HandleTypeDef *hfdcan, unsigned char *TxData, unsigned int StdId, unsigned int Length)
{
	return FDCAN_Send(hfdcan,StdId,"STD",TxData,"FD",Length,"OFF");
}
unsigned char FDCAN_SendData_Ext(FDCAN_HandleTypeDef *hfdcan, unsigned char *TxData, unsigned int ExtId, unsigned int Length, unsigned int Data_type)
{
		FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = ExtId;            
    TxHeader.IdType = FDCAN_EXTENDED_ID;    
    TxHeader.TxFrameType = Data_type; 
    TxHeader.DataLength = Length;      
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;          
		TxHeader.FDFormat = FDCAN_FD_CAN;            
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
        return 1; //发送
    return 0;
}





























































//	
//	
//    FDCAN_TxHeaderTypeDef TxHeader = {0};
//    TxHeader.Identifier = StdId;             /*32位 ID*/
//    TxHeader.IdType = FDCAN_STANDARD_ID;     /*标准ID*/
//    TxHeader.TxFrameType = FDCAN_DATA_FRAME; /*数据帧*/
//	if(Length == 0)
//		TxHeader.DataLength = FDCAN_DLC_BYTES_0;
//	else if(Length == 8)
//		TxHeader.DataLength = FDCAN_DLC_BYTES_8;      /*数据长度有专门的格式  FDCAN_DLC_BYTES_8*/
//    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
////	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           /*关闭速率切换*/
////  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            /*传统的CAN模式*/
//    TxHeader.BitRateSwitch = FDCAN_BRS_ON;           /*开启速率切换*/
//    TxHeader.FDFormat = FDCAN_FD_CAN;            /*FDCAN模式*/
//    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; /*无发送事件*/
//    TxHeader.MessageMarker = 0;

//    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
//        return 1; //发送
//    
//	return 0;


//    FDCAN_TxHeaderTypeDef TxHeader = {0};
//    TxHeader.Identifier = ExtId;             /*32位 ID*/
//    TxHeader.IdType = FDCAN_EXTENDED_ID;     /*拓展ID*/
//    TxHeader.TxFrameType = Data_type; /*帧类型*/
//    TxHeader.DataLength = Length;      /*数据长度有专门的格式*/
//		if(Length == 0)
//			TxHeader.DataLength = FDCAN_DLC_BYTES_0;
//		else if(Length == 8)
//			TxHeader.DataLength = FDCAN_DLC_BYTES_8;
//		else if(Length == 32)
//			TxHeader.DataLength = FDCAN_DLC_BYTES_32;
//    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
//    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           /*关闭速率切换*/
////    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            /*传统的CAN模式*/
//		TxHeader.FDFormat = FDCAN_FD_CAN;            /*FDCAN模式*/
//    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; /*无发送事件*/
//    TxHeader.MessageMarker = 0;
//    
//    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
//        return 1; //发送
//    return 0;
