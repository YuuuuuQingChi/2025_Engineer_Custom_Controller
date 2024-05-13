#include "ramp.h"
//
ramp_t chassis_vx_ramp=RAMP_GEN_DAFAULT;
ramp_t chassis_vy_ramp=RAMP_GEN_DAFAULT;
ramp_t chassis_vw_ramp=RAMP_GEN_DAFAULT;
ramp_t lift_ramp=RAMP_GEN_DAFAULT;
ramp_t stretch_1_ramp=RAMP_GEN_DAFAULT;
ramp_t stretch_2_ramp=RAMP_GEN_DAFAULT;

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