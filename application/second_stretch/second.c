#include "second.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "encoder.h"

#include "bmi088.h"

// static INS_Instance *gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *left2_speed_motor, *right2_speed_motor;
static Publisher_t *second_stretch_pub;                          // 二级应用消息发布者(二级反馈给cmd)
static Subscriber_t *second_stretch_sub;                         // cmd控制消息订阅者
static Second_Stretch_Upload_Data_s second_stretch_feedback_data; // 回传给cmd的二级状态信息
static Second_Stretch_Ctrl_Cmd_s second_stretch_cmd_recv;         // 来自cmd的控制信息

void Second_Stretch_Init()
{
    Motor_Init_Config_s second_stretch_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 50,
                .Ki            = 0,
                .Kd            = 0,
                // .DeadBand      = 0.1f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 10000,
            },
            .speed_PID = {
                .Kp            = 1,
                .Ki            = 0,
                .Kd            = 0,//0.0065,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut        = 7000,
            },
        },
        .controller_setting_init_config = {                                                                                                                                                                                                                              
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508};
    
    second_stretch_config.can_init_config.tx_id                             =3;
    second_stretch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    left2_speed_motor                                                            = DJIMotorInit(&second_stretch_config);

    second_stretch_config.can_init_config.tx_id                             =4;
    second_stretch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    right2_speed_motor                                                            = DJIMotorInit(&second_stretch_config);

    second_stretch_pub = PubRegister("second_stretch_feed", sizeof(Second_Stretch_Upload_Data_s));
    second_stretch_sub = SubRegister("second_stretch_cmd", sizeof(Second_Stretch_Ctrl_Cmd_s));
    
}


void Second_Stretch_Task()
{

    SubGetMessage(second_stretch_sub, &second_stretch_cmd_recv);

    DJIMotorEnable(left2_speed_motor);
    DJIMotorEnable(right2_speed_motor);

    switch (second_stretch_cmd_recv.second_stretch_mode) {
        // 停止
        case SECOND_STOP:
            DJIMotorStop(left2_speed_motor);
            DJIMotorStop(right2_speed_motor);
            break;
        default:
            break;
    }
    DJIMotorSetRef(left2_speed_motor, second_stretch_cmd_recv.left_now);
    DJIMotorSetRef(right2_speed_motor, second_stretch_cmd_recv.right_now);


    // 设置反馈数据,主要是imu和yaw的ecd
    second_stretch_feedback_data.new_left_angle  = left2_speed_motor->measure.total_angle;
    second_stretch_feedback_data.new_right_angle = right2_speed_motor->measure.total_angle;

    // 推送消息
    PubPushMessage(second_stretch_pub, (void *)&second_stretch_feedback_data);
}