#include "ramp.h"
//
ramp_t chassis_vx_ramp=RAMP_GEN_DAFAULT;
ramp_t chassis_vy_ramp=RAMP_GEN_DAFAULT;
ramp_t chassis_vw_ramp=RAMP_GEN_DAFAULT;
ramp_t lift_l_ramp=RAMP_GEN_DAFAULT;
ramp_t lift_r_ramp=RAMP_GEN_DAFAULT;
ramp_t stretch_left_ramp=RAMP_GEN_DAFAULT;
ramp_t stretch_right_ramp=RAMP_GEN_DAFAULT;
ramp_t BIG_PITCH_ramp = RAMP_GEN_DAFAULT;
ramp_t YAW_ramp = RAMP_GEN_DAFAULT;
ramp_t BIG_ROLL_ramp = RAMP_GEN_DAFAULT;
ramp_t SMALL_PITCH_ramp = RAMP_GEN_DAFAULT;
ramp_t SMALL_ROLL_ramp = RAMP_GEN_DAFAULT;
ramp_t GIMBAL_ramp = RAMP_GEN_DAFAULT;

void ramp_init(ramp_t *ramp, int32_t scale)
{
    ramp->count = 0;
    ramp->scale = scale;
}

float ramp_calc(ramp_t *ramp)
{
    if (ramp->scale <= 0)
        return 0;

    if (ramp->count++ >= ramp->scale)
        ramp->count = ramp->scale;

    ramp->out = ramp->count / ((float)ramp->scale);
    return ramp->out;
}