#include "CatchDelayCal.h"
#include "cmsis_os.h"

float TwoCar_Dis_Calc()
{
	float distance;
	float net_x = vision.field.carcenter_fieldinterp.x / 1000 + NET_TO_CENTER * cos(ang2rad(site.now.r));
	float net_y = vision.field.carcenter_fieldinterp.y / 1000 + NET_TO_CENTER * sin(ang2rad(site.now.r));
	distance = hypot(fabsf(net_x - send.R1_Exchange.pos.x / 1000), fabsf(net_y - send.R1_Exchange.pos.y / 1000));
	return distance;
}

SimResult_t simulateBallTrajectory(float initial_shoot_velocity)
{
    float target_y = NET_HEIGHT;
    BallState_t current_state;
    current_state.x = 0.0f;
    current_state.y = SHOOT_HEIGHT;
    current_state.time = 0.0f;

    float shoot_angle_rad = ang2rad(SHOOT_ANGLE);
    current_state.vx = initial_shoot_velocity * cosf(shoot_angle_rad);
    current_state.vy = initial_shoot_velocity * sinf(shoot_angle_rad);

    SimResult_t result = {.final_x = -1.0f, .time_s = -1.0f};

    while(current_state.time < MAX_SIM_DURATION){
        if(current_state.y <= target_y){
            float prev_y = current_state.y - current_state.vy * SIM_TIME_STEP;
            float prev_x = current_state.x - current_state.vx * SIM_TIME_STEP;
            float prev_time = current_state.time - SIM_TIME_STEP;

            if(fabsf(current_state.y - prev_y) < 0.001f){
                result.time_s = current_state.time;
                result.final_x = current_state.x;
            }
            else{
                float t_interp_factor = (target_y - prev_y) / (current_state.y - prev_y);
                result.time_s = prev_time + t_interp_factor * SIM_TIME_STEP;
                result.final_x = prev_x + t_interp_factor * current_state.vx * SIM_TIME_STEP; 
            }
        }
        break;
				
				float current_speed = hypotf(current_state.vx, current_state.vy);

			if(current_speed < SPEED_THRESHOLD){
					break;
			}

			float F_drag = K_DRAG * current_speed * current_speed;

			float ax_drag = -(F_drag * current_state.vx) / (current_speed * BALL_MASS);
			float ay_drag = -(F_drag * current_state.vy) / (current_speed * BALL_MASS);

			float ax_net = ax_drag;
			float ay_net = -GRAVITY_ACCEL + ay_drag;

			current_state.vx += ax_net * SIM_TIME_STEP;
			current_state.vy += ay_net * SIM_TIME_STEP;

			current_state.x += current_state.vx * SIM_TIME_STEP;
			current_state.y += current_state.vy * SIM_TIME_STEP;

			current_state.time += SIM_TIME_STEP;

			if(current_state.x > (TwoCar_Dis_Calc() * 1.5f) && current_state.y > target_y){
					break;
			}

			if(current_state.y < (SHOOT_HEIGHT - (NET_HEIGHT + VERTICAL_TOLERANCE)) &&
				 current_state.x < (TwoCar_Dis_Calc() * 0.5f)){
					break;
			}
    }

    if(current_state.time >= (MAX_SIM_DURATION - SIM_TIME_STEP)){
        result.final_x = -1.0f;
        result.time_s = -1.0f;
    }

    return result;
}

uint32_t CatchTimeCal()
{
    float target_distance = TwoCar_Dis_Calc();

    float min_v = 1.0f;
    float max_v = 30.0f;
    float v_step = 0.1f;
    int max_v_search_iterations = (int)((max_v - min_v) / v_step) + 1;

    float best_v_found = -1.0f;
    float best_time_s = -1.0f;
    float min_distance_error = FLT_MAX;

    for(int i = 0; i < max_v_search_iterations; ++i){
        float current_test_v = min_v + (float)i * v_step;

        SimResult_t sim_res = simulateBallTrajectory(current_test_v);

        if(sim_res.time_s >= 0.0f){
            float current_dis_error = fabsf(sim_res.final_x - target_distance);

            if(current_dis_error < min_distance_error){
                min_distance_error = current_dis_error;
                best_v_found = current_test_v;
                best_time_s = sim_res.time_s;
            }

            if(current_dis_error < HORIZONTAL_TOLERANCE){
                best_v_found = current_test_v;
                best_time_s = sim_res.time_s;
                break;
            }
        }
    }

    if(best_v_found > 0.0f && min_distance_error < HORIZONTAL_TOLERANCE){
        return (uint32_t)(best_time_s * 1000);
    }
    else{
        return 0xFFFFFFFF;
    }
}

