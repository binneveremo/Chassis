#include "Location.h"
#include "Encoder.h"
#include "Television.h"
#include <string.h>
#include "Global.h"
#include "math.h"
#include "Flow.h"

struct Site site;
void Location_Type_Choose(void){
	site.now.x = site.field.xfilter.position;
	site.now.y = site.field.yfilter.position;
	site.now.r = site.gyro.r;
	
	Field2Car(&site.car.xfilter.accel,&site.car.yfilter.accel,&site.field.xfilter.accel,&site.field.yfilter.accel);
	Field2Car(&site.car.xfilter.velocity,&site.car.yfilter.velocity,&site.field.xfilter.velocity,&site.field.yfilter.velocity);
}











