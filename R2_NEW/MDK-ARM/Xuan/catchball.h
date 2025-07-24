#ifndef CATCHBALL_H
#define CATCHBALL_H

#include "Interact.h"
#include "stdbool.h"
#include "HighTorque.h"
#include "Global.h"

#ifdef Carbon_Car
//#define R2_Fifth
#endif

//mechanism parameters
#define shoot_angle 65
#define height_diff 0.72755  //net_height(center)[1.90755m] - shoot_height[1.18m]
#define gravity_accel 9.7913
#define shooter_to_center 0.24
#define net_to_center 0.3223

#define NONFOLD_TIMEOUT_MS 120000
// --- Enums ---

// Overall state machine states
typedef enum
{
    STATE_INITIALIZE = 0,
    STATE_CATCHING_BALL = 1,
    STATE_DEFEND = 2,
    STATE_PRE_DUNK = 3,
    STATE_BACK_TO_FOLD = 4,
		STATE_TEST = 5,
		STATE_MIDCATCH = 6,
		STATE_OSCILLATE = 7,
		STATE_ERROR = 8,
    
} OverallState_t;

// Error codes
typedef enum
{
    ERROR_CODE_NONE = 0,
    ERROR_CODE_MOVE_TIMEOUT = 1,
    ERROR_CODE_COMM_LOST = 2,
    ERROR_CODE_OVER_LOAD = 3,
    ERROR_CODE_SELF_CHECK_FAILED = 4,
    ERROR_CODE_INVALID_STATE = 5
} ErrorCode_t;


// --- Main Control Status Structure ---
typedef struct
{
    // State Management
    OverallState_t current_state;
    OverallState_t previous_state; // Stores the state before entering STATE_ERROR

    // Error Handling
    ErrorCode_t error_code;
    bool in_recovery_period;       // Flag to indicate if system is in recovery period
		uint8_t recovery_count;

    // Flags for specific operational phases or conditions
    bool needs_self_check;          // True if Initialize state should perform self-check routine
    bool in_move_to_catching_ball;
    bool in_move_to_defend;
    bool in_move_to_pre_dunk;
    bool in_move_to_back_to_fold;
		bool in_move_to_test;
	  bool in_move_to_midcatch;
		bool in_oscillate;

    // Timing
    uint32_t move_start_time_ms;    // Timestamp for the start of a movement phase (for timeouts)
    uint32_t last_feedback_time_ms; // Timestamp of the last valid feedback update
		uint32_t nonfold_start_time;    // Timestamp of entering not-fold state

} CatchControlStatus_t;
extern CatchControlStatus_t catch_status;

// --- Timeout Definitions ---
// Updated to use new enum names
#define STATE_TIMEOUT_MS(st) \
    ( (st) == STATE_INITIALIZE ? 2000 : \
      (st) == STATE_CATCHING_BALL ? 5000 : \
      (st) == STATE_DEFEND ? 8000 : \
      (st) == STATE_PRE_DUNK ? 30000 : \
      (st) == STATE_BACK_TO_FOLD ? 5000 : \
			(st) == STATE_MIDCATCH ? 4000 : 0)

extern float filtered_pos;
extern float filtered_trq;
extern float filtered_pos_2nd;
extern float filtered_trq_2nd;
extern float Compensation_trq;

// Function Prototypes
void Loop_Judgement(void);
void Single_Control(void);
void Overall_Control(void);

// Helper functions
bool IsAtTargetPositionSettled(float target_pos);
float LowPassFilter(float input, float prev_output, float alpha);

// Error handling functions
void HandleError(ErrorCode_t error_code);
void ClearError(void);
bool IsInErrorState(void);
void RecoverFromError(void);
bool CheckMoveTimeout(OverallState_t state);



#endif // CATCHBALL_H
