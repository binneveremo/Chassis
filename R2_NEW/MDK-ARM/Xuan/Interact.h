#ifndef __INTERACT_H
#define __INTERACT_H
#include "HighTorque.h"
#include "catchball.h"
#include "Interact.h"
#include "Global.h"

struct Interact{
	enum {
		initial = 0,
		catch_ball = 1,
		defend = 2,
		predunk = 3,
		fold = 4,
		test = 5,
		midcatch = 6,
		oscillate = 7
	}defend_status;
	struct {
		char HT_Error;
	}wrongcode;
	struct {
		char R1_shooted;
		char dunk;
	}flagof;
	struct {
		struct Point self;
		struct Point shoot;
	}pos;
	

	
	
};
extern struct Interact interact;


#endif




