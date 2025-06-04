#ifndef __INTERACT_H
#define __INTERACT_H
#include "HighTorque.h"
#include "catchball.h"
#include "Interact.h"

struct Interact{
	enum {
		initial = 0,
		fold = 1,
		catch_ball = 2,
		defend = 3,
		predunk = 4,
		CMD_ERROR_CLEAR = 5,
		test = 6
	}defend_status;
	struct {
		
	}flagof;
	struct {
		char HT_Error;
	}wrongcode;
};
extern struct Interact interact;


#endif




