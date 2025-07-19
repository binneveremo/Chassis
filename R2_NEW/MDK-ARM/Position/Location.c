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
	
	Copy(interact.pos.self,site.now);
	Copy(interact.pos.shoot,shoot.receive.shoot_pos);
}
float vx_wheel_field,vy_wheel_field;
void WheelVelocity_Fuse_WithOdometer(void){
	Car2Field(&vx_wheel_field,&vy_wheel_field,&chassis.expect_status.front_velocity,&chassis.expect_status.left_velocity);
	static float wheel_total_velocity_history[5],odo_total_velocity_history[5];
	float wheel_velocity_variance = Variance_Cal(wheel_total_velocity_history,hypot(chassis.expect_status.front_velocity,chassis.expect_status.left_velocity),sizeof(wheel_total_velocity_history) / sizeof(float));
	float odo_velocity_variance   = Variance_Cal(odo_total_velocity_history,site.car.velocity_totalenc,sizeof(odo_total_velocity_history) / sizeof(float));
	float k = wheel_velocity_variance / (wheel_velocity_variance + odo_velocity_variance + 1e-3);
	site.field.vx_fuse = k * site.field.vx_enc + (1 - k) * vx_wheel_field;
	site.field.vy_fuse = k * site.field.vy_enc + (1 - k) * vy_wheel_field;
}










