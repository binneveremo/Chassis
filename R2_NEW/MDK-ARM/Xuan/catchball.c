#include "catchball.h"
#include "cmsis_os.h"
#include "math.h" 


// --- Global Variables and Parameters ---

HighTorque_t HighTorque[HIGHTORQUE_NUM] = {0};
//CatchStateFlag catch_sf = {0}; 
CatchControlStatus_t catch_status = { 
    .current_state = STATE_INITIALIZE, // Default initial state
    .previous_state = STATE_INITIALIZE, 
    .error_code = ERROR_CODE_NONE,
    .in_recovery_period = false,
    .needs_self_check = false,
    .in_move_to_catching_ball = false,
    .in_move_to_defend = false,
    .in_move_to_pre_dunk = false,
    .in_move_to_back_to_fold = false,
		.in_move_to_test = false,
		.in_move_to_midcatch = false,
		.in_oscillate = false,
    .move_start_time_ms = 0,
    .last_feedback_time_ms = 0
}; 


// Smooth transition parameters
const float SMOOTH_TRANSITION_RANGE = 15.0f;  // Range for smooth transition
const float MIN_SPEED_FACTOR = 0.2f;          // Minimum speed factor during transition 


// Current Control Commands to Motor Driver (Updated by Overall_Control, Used by Single_Control)
float State_Kp = 0;
float State_Kd = 0;
float State_Trq = 0; 
float State_Spd = 0;
float State_Pos = 0; 

//Thresholds
const float POS_THRESHOLD_MOVE_DONE = 6.0; 
const float POS_THRESHOLD_PREDUNK = 10.0;
const float POS_THRESHOLD_HOLD_SETTLED = 6.0; 
const float SPD_THRESHOLD_HOLD_SETTLED = 10.0;

static uint32_t catching_entry_time = 0;
const uint32_t CATCH_TIMEOUT_MS = 2000; 

// Add communication timeout constant
const uint32_t COMM_TIMEOUT_MS = 100; // 100ms timeout for communication loss

float filtered_pos = 35;
float filtered_trq = 0;
float filtered_pos_2nd = 0;
float filtered_trq_2nd = 0;
float Compensation_trq = 0;

float LowPassFilter(float input, float prev_output, float alpha)
{
	return alpha * input + (1 - alpha) * prev_output;
}

float LowPassFilter2nd(float input, float* prev_output, float* prev_output_2nd, float alpha)
{
	float first_stage = LowPassFilter(input, *prev_output, 0.02);
	*prev_output = first_stage;
	float second_stage = LowPassFilter(first_stage, *prev_output_2nd, alpha);
	*prev_output_2nd = second_stage;
	return second_stage;
}

// Target Positions
float Init_Pos = 1.219;
float CatchBall_Pos = 12.7;
float Defend_Pos = 122.94;
float PreDunk_Pos = 14.7;
float SelfCheck_Pos = 10;

// Control Parameters for Position HOLDING (Mapped to Overall_States enum indices)
// Indices:          {Initialize, CatchingBall, Defend, PreDunk, BackToFold, Test, MidCatch}
float Kp_Hold[7] =   {0,          0.4,          0.55,   0.45,    0.2,        0.55, 0.45 }; 
float Kd_Hold[7] =   {0,          0.06,         0.05,   0.04,    0,          0.05, 0.04 };      
float Trq_Hold[7]=   {0,          1.0,          3.0,    3.5,     0,          0,    1.0  };           
float Pos_Target[7]= {1.219,      12.7,         122.94, 14.7,    1.219,      41,   20   };

// Control Parameters for Velocity MOVEMENT (Based on original SpdUp/SpdDown)
float Spd_Move_Up = 130;
float Spd_Move_Up_Defend = 150; 
float Spd_Move_Up_Test = 30;
float Spd_Move_Down = -150; 
float Kd_Move_Up = 3;
float Kd_Move_Up_Test = 4;
float Kd_Move_Down = 0.2; 
float Trq_Move_Up = 0;  
float Trq_Move_Down = 0.5;



bool IsAtTargetPositionSettled(float target_pos)
{
    if (HIGHTORQUE_NUM == 0) return false; 

    float pos_err = HighTorque[0].fdbk.pos - target_pos;
    float current_spd = HighTorque[0].fdbk.spd; 

		if(catch_status.current_state != STATE_PRE_DUNK)
		{
			return fabsf(pos_err) < POS_THRESHOLD_HOLD_SETTLED && fabsf(current_spd) < SPD_THRESHOLD_HOLD_SETTLED;
		}
		else
		{
			return fabsf(pos_err) < POS_THRESHOLD_PREDUNK && fabsf(current_spd) < SPD_THRESHOLD_HOLD_SETTLED;
		}
}

bool CheckMoveTimeout(OverallState_t state)
{
    uint32_t timeout = STATE_TIMEOUT_MS(state);
    if (timeout == 0) return false;
    
    if (HAL_GetTick() - catch_status.move_start_time_ms > timeout) {
				interact.wrongcode.HT_Error = 2;
        HandleError(ERROR_CODE_MOVE_TIMEOUT);
        return true;
    }
    return false;
}

float TwoCar_Dis_Calc()
{
	float distance;
	float net_x = interact.pos.self.x / 1000 + net_to_center * cos(ang2rad(interact.pos.self.r));
	float net_y = interact.pos.self.y / 1000 + net_to_center * sin(ang2rad(interact.pos.self.r));
	distance = hypot(fabsf(net_x - interact.pos.shoot.x / 1000), fabsf(net_y - interact.pos.shoot.y / 1000));
	return distance;
}

uint32_t CatchTimeCal()
{
	uint32_t catch_delay_time;
//	catch_delay_time = sqrtf((2 * TwoCar_Dis_Calc() * tanf(ang2rad(65)) - height_diff) / gravity_accel);
	catch_delay_time = -0.009572 * TwoCar_Dis_Calc() * TwoCar_Dis_Calc() + 0.2574 * TwoCar_Dis_Calc() + 0.4732;
	return catch_delay_time;
}

// --- Error Handling Functions ---

void HandleError(ErrorCode_t error_code) {
		if(catch_status.current_state != STATE_ERROR)
		{
			catch_status.previous_state = catch_status.current_state;
		}
    catch_status.current_state = STATE_ERROR;
    catch_status.error_code = error_code;
    
    // Clear all movement flags
    catch_status.in_move_to_catching_ball = false;
    catch_status.in_move_to_defend = false;
    catch_status.in_move_to_pre_dunk = false;
    catch_status.in_move_to_back_to_fold = false;
		catch_status.in_move_to_midcatch = false;
    
    // Reset control commands
    State_Pos = 0;
    State_Kp = 0;
    State_Kd = 0;
    State_Trq = 0;
    State_Spd = 0;
}

void ClearError(void) {
    catch_status.error_code = ERROR_CODE_NONE;
		interact.wrongcode.HT_Error = 0;
}

bool IsInErrorState(void) {
    return catch_status.current_state == STATE_ERROR;
}

void RecoverFromError(void) {
    if (!IsInErrorState()) return;
    
    OverallState_t state_to_recover_to = catch_status.previous_state;
    
    ClearError();
		catch_status.current_state = state_to_recover_to;
    catch_status.move_start_time_ms = HAL_GetTick();
    catch_status.in_recovery_period = true;  // Set recovery period flag

    // Set appropriate movement flag based on recovery state
    switch(state_to_recover_to) {
        case STATE_CATCHING_BALL: 
            catch_status.in_move_to_catching_ball = true; 
            break;
        case STATE_DEFEND: 
            catch_status.in_move_to_defend = true; 
            break;
        case STATE_PRE_DUNK: 
            catch_status.in_move_to_pre_dunk = true; 
            break;
        case STATE_BACK_TO_FOLD: 
            catch_status.in_move_to_back_to_fold = true; 
            break;
				case STATE_MIDCATCH:
						catch_status.in_move_to_midcatch = true;
						break;
        default: 
            break;
    }
}

// --- Core Control Logic ---

void Single_Control()
{
    // Check for communication loss
    if (HIGHTORQUE_NUM == 0) {
        HandleError(ERROR_CODE_COMM_LOST);
        return;
    }

    // Update feedback timestamp if feedback data has changed
    static float last_pos = 0.0f;
    static float last_spd = 0.0f;
    static float last_trq = 0.0f;
    static float last_temp = 0.0f;

    if (HighTorque[0].fdbk.pos != last_pos ||
        HighTorque[0].fdbk.spd != last_spd ||
        HighTorque[0].fdbk.trq != last_trq ||
        HighTorque[0].fdbk.temp != last_temp) {
        catch_status.last_feedback_time_ms = HAL_GetTick();
        last_pos = HighTorque[0].fdbk.pos;
        last_spd = HighTorque[0].fdbk.spd;
        last_trq = HighTorque[0].fdbk.trq;
        last_temp = HighTorque[0].fdbk.temp;
    }

    // Check if feedback is too old
		if(!catch_status.needs_self_check){
			if (HAL_GetTick() - catch_status.last_feedback_time_ms > COMM_TIMEOUT_MS) {
					interact.wrongcode.HT_Error = 2;
					HandleError(ERROR_CODE_COMM_LOST);
					return;
			}}

    // Check for critical errors
		if(HighTorque[0].fdbk.temp > 50.0f)
		{
				interact.wrongcode.HT_Error = 1;
        HandleError(ERROR_CODE_OVER_LOAD);
        return;
    }

    HighTorque[0].ctrl = (HighTorque_ctrl_t){
        .pos = State_Pos,
        .spd = State_Spd,
        .trq = State_Trq,
        .Kp = State_Kp,
        .Kd = State_Kd
    };
}


void Overall_Control()
{
    static uint8_t init_step = 0; // For Initialize state sequence
    static uint32_t init_step_start_time = 0; // For tracking time within an init step
    const uint32_t INIT_SWEEP_UP_TIMEOUT = 1000; // ms timeout for sweeping up (example)
    const uint32_t INIT_SWEEP_DOWN_TIMEOUT = 3000; // ms timeout for sweeping down (example)
    const float INIT_SWEEP_UP_POS_CHECK = Init_Pos + 10; // Position check during sweep up (example)
    static uint32_t error_start = 0;

    // Handle error state recovery
    if (IsInErrorState()) {
        if (error_start == 0) {
            error_start = HAL_GetTick();
        }
        
        if (HAL_GetTick() - error_start > 3000) {
            RecoverFromError();
            error_start = 0;
        }
        return;
    }

    switch(catch_status.current_state)
    {
        case STATE_INITIALIZE: { 
            if (catch_status.needs_self_check) {
                static uint32_t init_step_start_time = 0;
                float target_pos = SelfCheck_Pos;
                switch(init_step)
                {
                    case 0:
                        State_Spd = Spd_Move_Up;
                        State_Kd = Kd_Move_Up;
                        State_Trq = Trq_Move_Up;
                        init_step_start_time = HAL_GetTick();
                        init_step = 1;
                        break;
                                
                    case 1:
                        if(HighTorque[0].fdbk.pos >= target_pos) {
                            init_step = 2;
                            init_step_start_time = HAL_GetTick();
                        }
                        else if(HAL_GetTick() - init_step_start_time > STATE_TIMEOUT_MS(STATE_INITIALIZE))
                        {
                            HandleError(ERROR_CODE_MOVE_TIMEOUT);
                        }
                        break;
                                    
                    case 2:
                        State_Spd = Spd_Move_Down;
                        State_Kd = Kd_Move_Down;
                        State_Trq = Trq_Move_Down;
                        if(fabsf(HighTorque[0].fdbk.pos - Pos_Target[STATE_INITIALIZE]) < POS_THRESHOLD_MOVE_DONE)
                        {
                            init_step = 3;
                        }
                        else if(HAL_GetTick() - init_step_start_time > STATE_TIMEOUT_MS(STATE_INITIALIZE))
                        {
                            HandleError(ERROR_CODE_MOVE_TIMEOUT);
                        }
                        break;
                                     
                    case 3:
                        catch_status.needs_self_check = false;
                        init_step = 0;
                        break;
                }
               
            } else {
                State_Pos = Pos_Target[STATE_BACK_TO_FOLD]; 
                State_Kp = Kp_Hold[STATE_BACK_TO_FOLD];
                State_Kd = Kd_Hold[STATE_BACK_TO_FOLD];
                State_Trq = Trq_Hold[STATE_BACK_TO_FOLD];
                State_Spd = 0;
                init_step = 0; 
            }
            break; } 

        case STATE_CATCHING_BALL: {
            float target_pos = Pos_Target[STATE_CATCHING_BALL];

            if (catch_status.in_move_to_catching_ball) {
                if(CheckMoveTimeout(STATE_CATCHING_BALL)) {
                    break; 
                }
                
                State_Kp = 0; // Velocity control

                // Determine movement direction and set parameters
                if (HighTorque[0].fdbk.pos > target_pos + POS_THRESHOLD_MOVE_DONE)
                {
                    State_Spd = Spd_Move_Down;
                    State_Kd = Kd_Move_Down;
                    State_Trq = Trq_Move_Down; //
                } 
                else if (HighTorque[0].fdbk.pos < target_pos - POS_THRESHOLD_MOVE_DONE) 
                {
                    State_Spd = Spd_Move_Up;
                    State_Kd = Kd_Move_Up;
                    State_Trq = Trq_Move_Up; //
                }

                else 
                {
                    State_Spd = 0; // Stop velocity command
                    catch_status.in_move_to_catching_ball = false;
                }

            } else {
                State_Pos = target_pos;
                State_Kp = Kp_Hold[STATE_CATCHING_BALL];
                State_Kd = Kd_Hold[STATE_CATCHING_BALL];
                State_Trq = Trq_Hold[STATE_CATCHING_BALL];
                State_Spd = 0;

                IsAtTargetPositionSettled(target_pos);
            }
            break; }

        case STATE_DEFEND: { 
            float target_pos = Pos_Target[STATE_DEFEND];

            if (catch_status.in_move_to_defend) {
                if(CheckMoveTimeout(STATE_DEFEND)) {
                    break;
                }
                State_Kp = 0; // Velocity control

                // Determine movement direction and set parameters
                if (HighTorque[0].fdbk.pos > target_pos + POS_THRESHOLD_MOVE_DONE) {
                    State_Spd = Spd_Move_Down;
                    State_Kd = Kd_Move_Down;
                    State_Trq = Trq_Move_Down;
                } else if (HighTorque[0].fdbk.pos < target_pos - POS_THRESHOLD_MOVE_DONE) {
                    State_Spd = Spd_Move_Up_Defend;
                    State_Kd = Kd_Move_Up;
                    State_Trq = Trq_Move_Up;
                }

                else {
                    State_Spd = 0; // Stop velocity command
                    catch_status.in_move_to_defend = false;
                }

            } else {
                State_Pos = target_pos;
                State_Kp = Kp_Hold[STATE_DEFEND];
                State_Kd = Kd_Hold[STATE_DEFEND];
                State_Trq = Trq_Hold[STATE_DEFEND];
                State_Spd = 0;

                IsAtTargetPositionSettled(target_pos);
            }
            break; }

        case STATE_PRE_DUNK: { 
            float target_pos = Pos_Target[STATE_PRE_DUNK]; 

            if (catch_status.in_move_to_pre_dunk) {
                if(CheckMoveTimeout(STATE_PRE_DUNK)) {
                    break;
                }
                 State_Kp = 0; // Velocity control

                // Determine movement direction and set parameters
                 if (HighTorque[0].fdbk.pos > target_pos + POS_THRESHOLD_PREDUNK) {
                    State_Spd = Spd_Move_Down;
                    State_Kd = Kd_Move_Down;
                    State_Trq = Trq_Move_Down;
                } else if (HighTorque[0].fdbk.pos < target_pos - POS_THRESHOLD_PREDUNK) {
                    State_Spd = Spd_Move_Up;
                    State_Kd = Kd_Move_Up;
                    State_Trq = Trq_Move_Up;
                }
                
                else {
                    State_Spd = 0; // Stop velocity command
                    catch_status.in_move_to_pre_dunk = false; 
                }

            } else {
                State_Pos = target_pos;
                State_Kp = Kp_Hold[STATE_PRE_DUNK];
                State_Kd = Kd_Hold[STATE_PRE_DUNK];
                State_Trq = Trq_Hold[STATE_PRE_DUNK];
                State_Spd = 0;

                IsAtTargetPositionSettled(target_pos);
            }
            break; }

        case STATE_BACK_TO_FOLD: {
            float target_pos = Pos_Target[STATE_BACK_TO_FOLD]; 

            if (catch_status.in_move_to_back_to_fold) {
                if(CheckMoveTimeout(STATE_BACK_TO_FOLD)) {
                    break;
                }
                 State_Kp = 0; // Velocity control

                // Determine movement direction and set parameters
                 if (HighTorque[0].fdbk.pos > target_pos + POS_THRESHOLD_MOVE_DONE) {
                    State_Spd = Spd_Move_Down;
                    State_Kd = Kd_Move_Down;
                    State_Trq = Trq_Move_Down;
                } else if (HighTorque[0].fdbk.pos < target_pos - POS_THRESHOLD_MOVE_DONE) {
                    State_Spd = Spd_Move_Up;
                    State_Kd = Kd_Move_Up;
                    State_Trq = Trq_Move_Up;
                }

                else {
                    State_Spd = 0; // Stop velocity command
                    catch_status.in_move_to_back_to_fold = false; 
                }

            } else {
                State_Pos = target_pos;
                State_Kp = Kp_Hold[STATE_BACK_TO_FOLD];
                State_Kd = Kd_Hold[STATE_BACK_TO_FOLD];
                State_Trq = Trq_Hold[STATE_BACK_TO_FOLD];
                State_Spd = 0; // Position control

                IsAtTargetPositionSettled(target_pos);
            }
            break; }

				
				case STATE_MIDCATCH: {
					float target_pos = Pos_Target[STATE_MIDCATCH];

            if (catch_status.in_move_to_midcatch) {
                if(CheckMoveTimeout(STATE_MIDCATCH)) {
                    break; 
                }
                
                State_Kp = 0; // Velocity control

                // Determine movement direction and set parameters
                if (HighTorque[0].fdbk.pos > target_pos + POS_THRESHOLD_MOVE_DONE)
                {
                    State_Spd = Spd_Move_Down;
                    State_Kd = Kd_Move_Down;
                    State_Trq = Trq_Move_Down; //
                } 
                else if (HighTorque[0].fdbk.pos < target_pos - POS_THRESHOLD_MOVE_DONE) 
                {
                    State_Spd = Spd_Move_Up;
                    State_Kd = Kd_Move_Up;
                    State_Trq = Trq_Move_Up; //
                }

                else 
                {
                    State_Spd = 0; // Stop velocity command
                    catch_status.in_move_to_midcatch = false;
                }

            } else {
                State_Pos = target_pos;
                State_Kp = Kp_Hold[STATE_MIDCATCH];
                State_Kd = Kd_Hold[STATE_MIDCATCH];
                State_Trq = Trq_Hold[STATE_MIDCATCH];
                State_Spd = 0;

                IsAtTargetPositionSettled(target_pos);
            }
					
					break;}
        
        case STATE_TEST: {
						float target_pos = Pos_Target[STATE_TEST];
						float start_pos = 35.0f;
						float current_pos = HighTorque[0].fdbk.pos;
						float distance_from_start = fabsf(current_pos - start_pos);

            if (catch_status.in_move_to_test) {
//                if(CheckMoveTimeout(STATE_DEFEND)) {
//                    break;
//                }
                State_Kp = 0; // Velocity control

        if(current_pos > target_pos + POS_THRESHOLD_MOVE_DONE)
				{
					State_Spd = Spd_Move_Down;
										
					if(distance_from_start <= 25.0f)
					{
						State_Kd = 0.2f;
					}
					else if(distance_from_start <= 70.0f)
					{
						State_Kd = 0.5f;
					}
				    else if(distance_from_start <= 100.0f)
					{
						State_Kd = 0.4f;
					}
					else
					{
						State_Kd = Kd_Move_Down;
					}
					State_Trq = 0;//Trq_Move_Down
					}
				else if(current_pos < target_pos - POS_THRESHOLD_MOVE_DONE)
				{
					State_Spd = Spd_Move_Up_Test;
					if(distance_from_start <= 25.0f)
					{
						State_Kd = 4.0f;
					}
					else if(distance_from_start <= 70.0f)
					{
						State_Kd = 7.5f;
					}
					else if(distance_from_start <= 100.0f)
					{
						State_Kd = 6.0f;
					}
					else
					{
						State_Kd = Kd_Move_Up_Test;
					}
					State_Trq = Trq_Move_Up;
				}
				else
				{
					State_Spd = 0;
					catch_status.in_move_to_test = false;
				}

            } else {
                State_Pos = target_pos;
                State_Kp = Kp_Hold[STATE_TEST];
                State_Kd = Kd_Hold[STATE_TEST];
                State_Trq = Trq_Hold[STATE_TEST];
                State_Spd = 0;

                IsAtTargetPositionSettled(target_pos);
            }
            break; }

        case STATE_OSCILLATE:{
					static uint8_t direction = 0; //0: to predunk, 1: to catching_ball
					float pos_catch = Pos_Target[STATE_CATCHING_BALL];
					float pos_predunk = Pos_Target[STATE_PRE_DUNK];
					float current_pos = HighTorque[0].fdbk.pos;
					float speed = 25.0f;
					
					//whether reach one side or not, switch direction
					if(direction == 0 && current_pos >= pos_predunk + 0.5){
						direction = 1;
					} else if(direction == 1 && current_pos <= pos_catch - 0.5){
						direction = 0;
					}
					
					State_Kp = 0;
					State_Kd = Kd_Move_Up + 2;
					State_Trq = 0;
					State_Spd = direction == 0 ? -speed : speed;
					
					break;
				}
				case STATE_ERROR: {
            // Error state is already handled before the switch statement
            break; }
				
				default: {
            // Handle unexpected state
            HandleError(ERROR_CODE_INVALID_STATE);
            break; }
    }
}


void Loop_Judgement()
{
    static uint32_t shoot_start_time;  // Track when R1_Shooted was set
    OverallState_t current_state = catch_status.current_state; // Read current state
    OverallState_t next_state = current_state; // Assume stay in current state by default

    // Check for R1_Shooted timing
    if (interact.flagof.R1_shooted) {
        if (shoot_start_time == 0) {
            shoot_start_time = HAL_GetTick();  // Start timing when flag is set
        } else if (HAL_GetTick() - shoot_start_time >= CatchTimeCal() * 1000) {  // Convert seconds to milliseconds
            interact.defend_status = catch_ball;  // Switch to catch state
						interact.flagof.R1_shooted = false;
            shoot_start_time = 0;  // Reset timer
        }
    } else {
        shoot_start_time = 0;  // Reset timer if flag is cleared
    }

    // If in recovery period, check if we should end it
    if (catch_status.in_recovery_period) {
        // End recovery period if we've been in the current state for more than 2 seconds
        if (HAL_GetTick() - catch_status.move_start_time_ms > 2000 && HAL_GetTick() - catch_status.last_feedback_time_ms <= COMM_TIMEOUT_MS) {
            catch_status.in_recovery_period = false;
						
						next_state = catch_status.previous_state;
        } else {
            return; // Skip state transition during recovery period
        }
    }

    if(next_state != STATE_ERROR)
    {
        // --- State Transition Logic ---
        switch(interact.defend_status)
        {
            case initial:
                next_state = STATE_INITIALIZE;
                break;

            case fold:
                next_state = STATE_BACK_TO_FOLD;
                break;

            case catch_ball:
                next_state = STATE_CATCHING_BALL;
                break;

            case defend:
                next_state = STATE_DEFEND;
                break;

            case predunk:
                next_state = STATE_PRE_DUNK;
                break;
            
            case test:
                next_state = STATE_TEST;
                break;
						
						case midcatch:
							  next_state = STATE_MIDCATCH;
								break;
						
						case oscillate:
								next_state = STATE_OSCILLATE;
								break;
                        
            default:
                HandleError(ERROR_CODE_INVALID_STATE);
                break;
        }
    }
    // --- Apply the determined next state and Handle State Entry ---
    if (next_state != current_state) {
				 
         // Clear all 'InMoveTo' flags when leaving *any* state
         catch_status.in_move_to_catching_ball = false;
         catch_status.in_move_to_defend = false;
         catch_status.in_move_to_pre_dunk = false;
         catch_status.in_move_to_back_to_fold = false;
				 catch_status.in_move_to_test = false;
				 catch_status.in_move_to_midcatch = false;
				 catch_status.in_oscillate = false;

         // Apply the new state
        catch_status.current_state = next_state;
        catch_status.move_start_time_ms = HAL_GetTick(); // Set timer for the new state's movement

         // --- State Entry Actions ---
         switch(next_state) {
             case STATE_INITIALIZE:
                // catch_status.needs_self_check = true;
                break;
             case STATE_CATCHING_BALL:
                catch_status.in_move_to_catching_ball = true; 
                break;
             case STATE_DEFEND:
                catch_status.in_move_to_defend = true; 
                break;
             case STATE_PRE_DUNK:
                catch_status.in_move_to_pre_dunk = true; 
                break;
             case STATE_BACK_TO_FOLD:
                catch_status.in_move_to_back_to_fold = true;
                break;
             case STATE_ERROR:
                // Error state entry is handled by HandleError function
                break;
						 case STATE_TEST:
							 catch_status.in_move_to_test = true;
								break;
						 case STATE_MIDCATCH:
							 catch_status.in_move_to_midcatch = true;
							  break;
						 case STATE_OSCILLATE:
							 catch_status.in_oscillate = true;
								break;
						 
             default:
                // Handle unexpected state
                HandleError(ERROR_CODE_INVALID_STATE);
                break;
         }
    }
}
