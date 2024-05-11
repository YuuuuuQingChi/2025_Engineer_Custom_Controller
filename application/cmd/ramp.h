#ifndef EED30425_5AE1_4657_A58C_29167E54510E
#define EED30425_5AE1_4657_A58C_29167E54510E
#ifndef RAMP_H_
#define RAMP_H_


#include "main.h"
#include "rm_referee.h"

#define bool  _Bool
#define true  1
#define false 0

#define RAMP_GEN_DAFAULT \
    {                    \
        0,               \
            0,           \
            0,           \
    }

#define RAMP_TIME 500

// 斜坡类型，计算WASD移动映射在底盘的速度
typedef struct ramp_t
{
  int32_t count; //计数值
  int32_t scale; //规模
  float   out; //输出
}ramp_t;


void ramp_init(ramp_t *ramp, int32_t scale);

extern ramp_t chassis_vx_ramp;
extern ramp_t chassis_vy_ramp;
extern ramp_t chassis_vw_ramp;
extern ramp_t lift_ramp;
extern ramp_t stretch_1_ramp;
extern ramp_t stretch_2_ramp;
float ramp_calc(ramp_t *ramp);

#endif

#endif /* EED30425_5AE1_4657_A58C_29167E54510E */
