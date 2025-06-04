#pragma once
#include "mngCommu.h"

struct Game_Pad_Data {
	float rocker[4];
	unsigned char key[22];
	unsigned char last[22];
	unsigned char witch[10];
	char Debug_Page;
};

extern struct Game_Pad_Data GamePad_Data;

#define GamePad_Init() Commu_init()
#define GamePadKey_FallingCheck(index) ((GamePad_Data.key[index] == 0) && (GamePad_Data.last[index] == 1))

void Get_GamePad_Data(void);
void GamePad_Data_Cla(void);
float Char2float(char str[4]);

