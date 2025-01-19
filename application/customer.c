#include "customer.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "encoder.h"
#include "LKmotor.h"

static DJIMotorInstance *joint4;
static LKMotorInstance *joint1,*joint3;

void ARM_Init()
{
    //   Motor_Init_Config_s joint4_config = {
    //     .can_init_config = {
    //         .can_handle = &hfdcan3,
    //     },
    //     // .controller_param_init_config = {
    //     //     .speed_PID = {
    //     //         .Kp            = 1.2, // 4.5
    //     //         .Ki            = 0.1,   // 0
    //     //         .Kd            = 0,   // 0
    //     //         .IntegralLimit = 200,
    //     //         .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //     //         .MaxOut        = 12000,
    //     //     },
    //     // },
    //     // .controller_setting_init_config = {
    //     //     .angle_feedback_source = MOTOR_FEED,
    //     //     .speed_feedback_source = MOTOR_FEED,
    //     //     .outer_loop_type       = SPEED_LOOP,
    //     //     .close_loop_type       = SPEED_LOOP,
    //     //     .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
    //     // },
    //     .motor_type = M3508};
    // joint4_config.can_init_config.tx_id    = 1;
    // joint4 = DJIMotorInit(&joint4_config);
    // DJIMotorStop(joint4);

    Motor_Init_Config_s joint1_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
            .rx_id      = 2,
            .tx_id      = 2
        },
        .motor_type = LK_MS9015,
        .offset = 0,
        .is_reserved = false 
        };
    joint1 = LKMotorInit(&joint1_config);

    Motor_Init_Config_s joint3_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
            .rx_id      = 3,
            .tx_id      = 3
        },
        .motor_type = LK_MS7015V3,
        .offset = 27720,
        .is_reserved = false 
        };
    joint3 = LKMotorInit(&joint3_config);

}
void ARM_Control()
{
   LKMotorSetSpeed(joint3,0);

}