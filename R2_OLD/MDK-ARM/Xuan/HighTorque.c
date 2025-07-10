#include "HighTorque.h"
#include "Can_Bsp.h"
#include <string.h>

#define ABS(x) fabsf(x)

/*HighTorque_motor_t HTDW_4538_32_NE = {
                       //.GR = 32,
                       .torque_limit = 18,
                       .spd_limit = 100,
                       //.trq_k_f = 0.414104,
                       //.trq_b_f = -0.472467
},
                   HTDW_5047_36_NE = {.GR = 36,
                                      .torque_limit = 20,
                                      .spd_limit = 90};*/

//HighTorque_t HighTorque[HIGHTORQUE_NUM + 1];

unsigned char TxData[32] = {HIGHTORQUE_DATA_W | HIGHTORQUE_DATA_TYPE_8 | 1,
                                HIGHTORQUE_REG_MODE,
                                HIGHTORQUE_MODE_POS,
                                HIGHTORQUE_DATA_W | HIGHTORQUE_DATA_TYPE_FLOAT | HIGHTORQUE_MODE2,
                                5,
                                HIGHTORQUE_REG_POS_CTRL};
void HighTorque_SendPosParam_f(void *FDCAN_handle, unsigned char ID)
{
    unsigned char ID_array = ID == HIGHTORQUE_ADDR_BCAST ? HIGHTORQUE_NUM : ID - HIGHTORQUE_IDOFFSET;

    *(float *)&TxData[6] = HighTorque[ID_array].ctrl.pos / 360;

    *(float *)&TxData[10] = HighTorque[ID_array].ctrl.spd / 360;

    *(float *)&TxData[14] = /*HTDW_motor ? (HighTorque[ID_array].ctrl.trq - HTDW_motor->trq_d) / HTDW_motor->trq_k :*/ HighTorque[ID_array].ctrl.trq;

    *(float *)&TxData[18] = HighTorque[ID_array].ctrl.Kp;
    *(float *)&TxData[22] = HighTorque[ID_array].ctrl.Kd;

    TxData[26] = HIGHTORQUE_DATA_R | HIGHTORQUE_DATA_TYPE_FLOAT | 3;
    TxData[27] = HIGHTORQUE_REG_POS_FDBK;
    TxData[28] = HIGHTORQUE_DATA_R | HIGHTORQUE_DATA_TYPE_FLOAT | 1;
    TxData[29] = HIGHTORQUE_REG_TEMP;

    memset(&TxData[30], HIGHTORQUE_NOP, 32 - 30);


		FDCAN_SendData_Ext(FDCAN_handle, TxData, HIGHTORQUE_ADDR_RE | ID, FDCAN_DLC_BYTES_32,FDCAN_DATA_FRAME);
}

void HighTorque_Stop(void *FDCAN_handle, unsigned char ID)
{
    unsigned char TxData[5] = {HIGHTORQUE_DATA_W | HIGHTORQUE_DATA_TYPE_8 | 1,
                               HIGHTORQUE_REG_MODE,
                               HIGHTORQUE_MODE_STOP,
                               HIGHTORQUE_DATA_R | HIGHTORQUE_DATA_TYPE_8 | 1,
                               HIGHTORQUE_REG_MODE};

	FDCAN_SendData(FDCAN_handle, TxData, HIGHTORQUE_ADDR_RE | ID, 5);
}
void HighTorque_SetSpdLimit(void *FDCAN_handle, unsigned char ID, float spd, float acc)
{
    unsigned char TxData[12] = {HIGHTORQUE_DATA_W | HIGHTORQUE_DATA_TYPE_FLOAT | 2,
                                HIGHTORQUE_REG_SPD_LIMIT};

    *(float *)&TxData[2] = ABS(spd);
    *(float *)&TxData[6] = ABS(acc);

    TxData[10] = HIGHTORQUE_DATA_R | HIGHTORQUE_DATA_TYPE_FLOAT | 2;
    TxData[11] = HIGHTORQUE_REG_SPD_LIMIT;

	FDCAN_SendData(FDCAN_handle, TxData, HIGHTORQUE_ADDR_RE | ID, FDCAN_DLC_BYTES_12);
}

void HighTorque_SwitchMode(void *FDCAN_handle, unsigned char ID, unsigned char HIGHTORQUE_MODE)
{
    unsigned char ID_array = ID == HIGHTORQUE_ADDR_BCAST ? HIGHTORQUE_NUM : ID;

    unsigned char TxData[8] = {HIGHTORQUE_DATA_W | HIGHTORQUE_DATA_TYPE_8 | 1,
                               HIGHTORQUE_REG_MODE,
                               HIGHTORQUE_MODE,
                               HIGHTORQUE_DATA_R | HIGHTORQUE_DATA_TYPE_FLOAT | HIGHTORQUE_MODE2,
                               3,
                               HIGHTORQUE_REG_POS_FDBK};

    memset(&TxData[6], HIGHTORQUE_NOP, 8 - 6);

	FDCAN_SendData(FDCAN_handle, TxData, HIGHTORQUE_ADDR_RE | ID, 8);
}

/*void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t RxFifo0[64];
    FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxHeader, RxFifo0);

    switch ((unsigned long)hfdcan->Instance)
    {
    case (unsigned long)FDCAN3:
    {
        unsigned char ID_array = (FDCAN_RxHeader.Identifier >> 8) - HIGHTORQUE_ID_OFFSET;

        if (RxFifo0[0] == (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | HIGHTORQUE_MODE2) &&
            RxFifo0[1] == 3 &&
            RxFifo0[2] == HIGHTORQUE_REG_POS_FDBK)
        {
            HighTorque[ID_array].fdbk.pos = *(float *)&RxFifo0[3] * 360;
            HighTorque[ID_array].fdbk.spd = *(float *)&RxFifo0[7] * 360;
            // HighTorque[ID_array].fdbk.trq = *(float *)&RxFifo0[7] * HTDW_4538_32_NE.trq_k_f + HTDW_4538_32_NE.trq_b_f;
            HighTorque[ID_array].fdbk.trq = *(float *)&RxFifo0[7];
        }
        break;
    }
    }
}*/
unsigned char HT_Data[20];
int HT_ID;
void HighTorque_getFedback(FDCAN_RxHeaderTypeDef* FDCAN_RxHeader, uint8_t* RxData)
{
	if(RxData[0] != (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | 3)) 
		return;
	if(RxData[1] != HIGHTORQUE_REG_POS_FDBK)
		return;
	if(RxData[14] != (HIGHTORQUE_DATA_RE | HIGHTORQUE_DATA_TYPE_FLOAT | 1))
		return;
	if(RxData[15] != HIGHTORQUE_REG_TEMP)
	  return;
	HT_ID = (FDCAN_RxHeader->Identifier >> 8) - HIGHTORQUE_IDOFFSET;
	memcpy(HT_Data,RxData,20);
	
    
}
void HT_Test(void){
	HighTorque[HT_ID].fdbk.pos = *(float *)&HT_Data[2];
	HighTorque[HT_ID].fdbk.pos *= 360;
	HighTorque[HT_ID].fdbk.spd = *(float *)&HT_Data[6];
	HighTorque[HT_ID].fdbk.spd *= 360;
	HighTorque[HT_ID].fdbk.trq = *(float *)&HT_Data[10];
	HighTorque[HT_ID].fdbk.temp = *(float *)&HT_Data[16];
}



//// HighTorque[ID_array].fdbk.trq = *(float *)&RxData[10] * HTDW_4538_32_NE.trq_k + HTDW_4538_32_NE.trq_d;