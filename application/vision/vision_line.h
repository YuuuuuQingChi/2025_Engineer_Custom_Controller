#ifndef _VISION_LINE_H
#define _VISION_LINE_H

#include "message_center.h"
#include "general_def.h"

void vision_recv_callback();

void Vision_Int();

void Vision_Task();

extern RC_ctrl_t *vision_rc_data;

#endif // !_VISION_LINE_H