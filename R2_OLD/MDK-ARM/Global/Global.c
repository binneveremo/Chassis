#include "Global.h"
char Car_Color;


float char2float(unsigned char * data){
	float num;
	memcpy((unsigned char *)&num,data,sizeof(float));
	return num;
}
struct Point Merge_Point(struct Point a,struct Point b){
	struct Point p;
	Copy(p,a);
	Copy(p.r,b.r);
	return p;
}






