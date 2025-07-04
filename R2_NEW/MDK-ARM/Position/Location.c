#include "Location.h"
#include "Encoder.h"
#include "Television.h"
#include <string.h>
#include "Global.h"
#include "math.h"
#include "Flow.h"

struct Site site;
void Location_Type_Choose(void){
	Copy(site.now,vision.field.carcenter_fieldinterp);
	site.now.r = site.gyro.r;
}
///////////////////////////////////////////////////////码盘与陀螺仪速度融合计算///////////////////////////////////////////////////////////////
//码盘的y轴是陀螺仪的-x轴
void Enc_VXVY_Fuse_With_Gyro_AXAY(float dt){
	site.car.velocity_totalgyro = hypot(site.car.vx_gyro,site.car.vy_gyro);
	site.field.vx_gyro  = site.car.vx_gyro * cos(ang2rad(site.now.r)) - site.car.vy_gyro * sin(ang2rad(site.now.r));
	site.field.vy_gyro  = site.car.vx_gyro * sin(ang2rad(site.now.r)) + site.car.vy_gyro * cos(ang2rad(site.now.r));
}












