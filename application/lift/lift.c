#include "lift.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "encoder.h"

static DJIMotorInstance *lift_left_motor, *lift_right_motor;
static Publisher_t *lift_pub;                          // 升降应用消息发布者(升降反馈给cmd)
static Subscriber_t *lift_sub;                         // cmd控制消息订阅者
static Lift_Upload_Data_s lift_feedback_data; // 回传给cmd的升降状态信息
static Lift_Ctrl_Cmd_s lift_cmd_recv;         // 来自cmd的控制信息

void Lift_Init()
{
// 升降左电机
    Motor_Init_Config_s lift_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 250,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit =3000,
                .MaxOut = 15000,
            },
            .speed_PID = {
                .Kp            = 0.5,
                .Ki            = 0,
                .Kd            = 0.005,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut        = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    lift_config.can_init_config.tx_id    = 5;
    lift_left_motor = DJIMotorInit(&lift_config);

    lift_config.can_init_config.tx_id      = 6;
    lift_right_motor  = DJIMotorInit(&lift_config);

    lift_pub = PubRegister("lift_feed", sizeof(Lift_Upload_Data_s));
    lift_sub = SubRegister("lift_cmd", sizeof(Lift_Ctrl_Cmd_s));
}

void Lift_Task()
{

    SubGetMessage(lift_sub, &lift_cmd_recv);
    switch (lift_cmd_recv.lift_mode) {
        case LIFT_STOP:
            DJIMotorStop(lift_left_motor);
            DJIMotorStop(lift_right_motor);
            break;
        case LIFT: 
            DJIMotorEnable(lift_left_motor);
            DJIMotorEnable(lift_right_motor);
            DJIMotorSetRef(lift_left_motor, lift_cmd_recv.left_angle); 
            DJIMotorSetRef(lift_right_motor, lift_cmd_recv.right_angle);
            break;
        default:
            break;
    }

    lift_feedback_data.now_left_angle  = lift_left_motor->measure.total_angle;
    lift_feedback_data.now_right_angle = lift_right_motor->measure.total_angle;

    lift_feedback_data.now_left_current = lift_left_motor->measure.real_current;
    lift_feedback_data.now_right_current = lift_right_motor->measure.real_current;

    lift_feedback_data.now_left_speed = lift_left_motor->measure.speed_aps;
    lift_feedback_data.now_right_speed = lift_right_motor->measure.speed_aps;

    // 推送消息
    PubPushMessage(lift_pub, (void *)&lift_feedback_data);
}