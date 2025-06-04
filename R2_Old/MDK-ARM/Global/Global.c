#include "Global.h"
char Car_Color;


float char2float(unsigned char * data){
	float num;
	memcpy((unsigned char *)&num,data,sizeof(float));
	return num;
}







