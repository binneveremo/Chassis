#ifndef __MINE_H
#define __MINE_H

#include "mngCommu.h"
#include "Chassis.h"
#define GamePad_Init() Commu_init()

struct Game_Pad_Data{
	float rocker[4];
	unsigned char key[22];
	unsigned char last[22];
	unsigned char witch[10];
	char Debug_Page;
};
extern struct Game_Pad_Data GamePad_Data;

void Get_GamePad_Data(void);
void GamePad_Data_Cla(void);
float Char2float(char str[4]);
#endif
