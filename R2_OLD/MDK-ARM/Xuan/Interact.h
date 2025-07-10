#ifndef __INTERACT_H
#define __INTERACT_H
#include "HighTorque.h"
#include "catchball.h"
#include "Interact.h"

struct Interact{
	enum {
		initial = 0,
		catch_ball = 1,
		defend = 2,
		predunk = 3,
		fold = 4,
		test = 5,
		moving = 6,
		oscillate = 7
	}defend_status;
	struct {
		char HT_Error;
	}wrongcode;
};
extern struct Interact interact;


#endif




