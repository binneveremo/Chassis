#include "Flash_Bsp.h"
#include "stm32h7xx_hal.h"
#include "stdbool.h"


#define SIZE_OF_FLASHWORD 32   //每个flash word是32byte
#define Flash_Error() flash.flagof.error = true;

FlashBsp_t flash;


void FlashBsp_Init(void){	
  HAL_FLASH_Unlock();
  volatile unsigned char* from = (unsigned char*)(0x08000000 + (unsigned int)(flash.sector)*128*1024);
  for(unsigned int i = 0; i < FLASHBSP_SRAMBUF_SIZE; i++) 
		flash.sram_buf[i] = from[i];
	HAL_FLASH_Lock();
}

void FlashBsp_Program(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = flash.sector;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Banks = FLASH_BANK_1;
    unsigned int SectorError = 0;
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
       Flash_Error();
	  FLASH_WaitForLastOperation(1000, FLASH_BANK_1);
    __IO unsigned char* dest = (unsigned char*)(0x08000000 + (unsigned int)(flash.sector)*128*1024);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_SNECCERR | FLASH_FLAG_WRPERR | FLASH_FLAG_BSY);
    for(unsigned int i = 0; i < FLASHBSP_SRAMBUF_SIZE / SIZE_OF_FLASHWORD; i++)
    {
		if(HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD, 
            (unsigned int)(dest + i * SIZE_OF_FLASHWORD), 
            (unsigned int)(flash.sram_buf + i * SIZE_OF_FLASHWORD)) != HAL_OK && 
           HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD, 
            (unsigned int)(dest + i * SIZE_OF_FLASHWORD), 
            (unsigned int)(flash.sram_buf + i * SIZE_OF_FLASHWORD)  != HAL_OK))
		{
            Flash_Error();
		}
	}
	HAL_FLASH_Lock();
	FlashBsp_Init();
}
void FlashBsp_Write(unsigned char* data, unsigned int sram_addr, unsigned int size)
{
    if(sram_addr + size > FLASHBSP_SRAMBUF_SIZE)
        Flash_Error();
    for(unsigned int i = 0; i < size; i++)
        flash.sram_buf[sram_addr + i] = data[i];
}
void FlashBsp_Read(unsigned char* data, unsigned int sram_addr, unsigned int size)
{
    if(sram_addr + size > FLASHBSP_SRAMBUF_SIZE)
       Flash_Error();
    for(unsigned int i = 0; i < size; i++)
        data[i] = flash.sram_buf[sram_addr + i];
}

void FlashBsp_Erase(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = flash.sector;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	  EraseInitStruct.Banks = FLASH_BANK_1;
    unsigned int SectorError = 0;
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
       Flash_Error();
    HAL_FLASH_Lock();
    FlashBsp_Init();
}
