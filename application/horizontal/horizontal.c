#include "horizontal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "referee_init.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"


static Publisher_t *Horizontal_pub;                   
static Subscriber_t *Horizontal_sub;                   

static Horizontal_Ctrl_Cmd_s Horizontal_cmd_recv;        
static Horizontal_Upload_Data_s Horizontal_feedback_data; 

static DJIMotorInstance *motor_Horizontal;


void Horizontal_Init()
{

    Motor_Init_Config_s Horizontal_motor_config = {
        .can_init_config.can_handle   = &hfdcan3,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            =250, // 4.5
                .Ki            = 0,  // 0
                .Kd            = 0,  // 0
                .IntegralLimit = 3000,
                //@？？？这是啥
                //.Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                /////////////////////////////////////////没看懂
                .MaxOut        = 18000,
                0},
            .speed_PID = {
                .Kp            = 0.35, // 0.4
                .Ki            = 0,   // 0
                .Kd            = 0.01,
                .IntegralLimit = 3000,
                //.Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 3000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508,
    };

    Horizontal_motor_config.can_init_config.tx_id                             = 4;

    motor_Horizontal                                                            = DJIMotorInit(&Horizontal_motor_config);
    Horizontal_pub = PubRegister("Horizontal_feed", sizeof(Horizontal_Upload_Data_s));
    Horizontal_sub = SubRegister("Horizontal_cmd", sizeof(Horizontal_Ctrl_Cmd_s));

}

/* 机器人底盘控制核心任务 */
void Horizontal_Task()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
    SubGetMessage(Horizontal_sub, &Horizontal_cmd_recv);
switch (Horizontal_cmd_recv.Horizontal_mode) {
        case HORIZONTAL_MOVE:
        DJIMotorEnable(motor_Horizontal);
            break;
        default:
            break;
    }   
    DJIMotorSetRef(motor_Horizontal, Horizontal_cmd_recv.Now_MechAngle);

    // 推送反馈消息
    Horizontal_feedback_data.Horizontal_Movement = motor_Horizontal->measure.total_angle;
    PubPushMessage(Horizontal_pub, (void *)&Horizontal_feedback_data);
}
