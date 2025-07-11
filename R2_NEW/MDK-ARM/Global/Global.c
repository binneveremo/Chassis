#include "Global.h"

struct Point Merge_Point(struct Point a,struct Point b){
	struct Point p;
	Copy(p,a);
	Copy(p.r,b.r);
	return p;
}

float Average(float * array,int float_size){
	float total;
	for(unsigned int i = 0; i < float_size; i++)
		total += array[i];
	return total / float_size;
}

float Variance_Cal(float * buffer,float data,int float_size){
	for(unsigned int i = 0;i < float_size - 1; i++)
		buffer[i] = buffer[i + 1];
	buffer[float_size - 1] = data;
	float mean = Average(buffer,float_size),total = 0;
	for(unsigned int i = 0;i < float_size; i++)
		total += (buffer[i] - mean) * (buffer[i] - mean);
	return total / float_size;
}





