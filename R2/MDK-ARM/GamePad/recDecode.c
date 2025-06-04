#include "recDecode.h"
#include "mngCommu.h"
//#include "Config.h"
#include <string.h>

struct receivedPackage recPkg;
static uint8_t fks[50],sws[50];

// 返回数据包类型标识符
uint8_t recDecodeData(uint8_t *pdata) {
	memcpy(&recPkg, pdata, sizeof(recPkg));
	return 1;
}

int8_t get_rocker(uint8_t n) {
	return recPkg.rockers[n];
}
uint8_t get_fk_state(uint8_t n) {
	if(recPkg.FunctionKeys >> n & 0x00000001)
	{
		if(fks[n] < 1)
		{
			fks[n] += 1;
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		fks[n] = 0;
		return 0;
	}
}
uint8_t get_sw_state(uint8_t n) {
	if(recPkg.switchs >> n & 0x00000001)
	{
		if(sws[n] < 1)
		{
			sws[n] += 1;
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		sws[n] = 0;
		return 0;
	}
}


/* 从数据包获取参数的函数 */
uint8_t get_crc(uint8_t *data) {
	return *(data + 0);
}
uint8_t get_lenth(uint8_t *data) {
	return *(data + 1);
}
uint8_t get_pkgIndex() {
 	return recPkg.send_index;
}
