#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "encoder.h"
#include "first.h"
#include "controller.h"
#include "remote_control.h"
#include "robot_cmd.h"

static DJIMotorInstance *forward_left_motor, *forward_right_motor;
static EncoderInstance_s *left_angle, *forward_angle;
static Publisher_t *forward_pub;                          // 一级应用消息发布者(一级反馈给cmd)
static Subscriber_t *forward_sub;                         // cmd控制消息订阅者
static Forward_Upload_Data_s forward_feedback_data; // 回传给cmd的一级状态信息
static Forward_Ctrl_Cmd_s forward_cmd_recv;         // 来自cmd的控制信息
PIDInstance *encoder_pid;
float output_left;
float output_right;
float speed_left;
float speed_right;
void Forward_Init()
{
    Encoder_Init_Config_s encoder_config; 
    encoder_config.can_init_config.can_handle = &hfdcan3;
    encoder_config.encoder_type = MT6825;
    encoder_config.can_init_config.rx_id = 0x003;
    left_angle                     = EncoderInit(&encoder_config);
    encoder_config.can_init_config.rx_id = 0x004;
    forward_angle                  = EncoderInit(&encoder_config);
   
    PID_Init_Config_s encoder_pid_config = {
        

                          .Kp            = 135.5, // 0
                          .Ki            = 22.5, // 0
                          .Kd            = 8, // 0
                          .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                          .IntegralLimit = 4500,
                          .MaxOut        = 9000, // 20000
                      };
                      
    encoder_pid = PIDRegister(&encoder_pid_config);
    // 左电机
    Motor_Init_Config_s forward_left_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
            .tx_id      = 3,
        },
        .controller_param_init_config = {
            
            .speed_PID = {
                .Kp            = 2.04,     
                .Ki            = 0.25,
                .Kd            = 0,
                .IntegralLimit = 4000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .MaxOut        = 11000,
            },
            
        },
        .controller_setting_init_config = {
            .speed_feedback_source = MOTOR_FEED,
            .close_loop_type       = SPEED_LOOP,
            .outer_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M2006};
    // 右电机
    Motor_Init_Config_s forward_right_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
            .tx_id      = 2,
        },
        .controller_param_init_config = {
            
            .speed_PID = {
                .Kp            = 2.16,
                .Ki            = 0.43,
                .Kd            = 0,
                .IntegralLimit = 4000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,

                .MaxOut        = 11000,
            },
        },
        .controller_setting_init_config = {
            .speed_feedback_source = MOTOR_FEED,
            .close_loop_type       = SPEED_LOOP,
            .outer_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M2006,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    forward_left_motor = DJIMotorInit(&forward_left_config);
    forward_right_motor  = DJIMotorInit(&forward_right_config);

    
   
    forward_pub = PubRegister("forward_feed", sizeof(Forward_Upload_Data_s));
    forward_sub = SubRegister("forward_cmd", sizeof(Forward_Ctrl_Cmd_s));

}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void Forward_Task()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(forward_sub, &forward_cmd_recv);
    DJIMotorEnable(forward_left_motor);
    DJIMotorEnable(forward_right_motor);
    DJIMotorSetRef(forward_left_motor, forward_cmd_recv.angel_output); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    DJIMotorSetRef(forward_right_motor, forward_cmd_recv.angel_output1);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (forward_cmd_recv.Forward_mode) {
        // 停止
        case FORWARD_STOP:
            DJIMotorStop(forward_left_motor);
            DJIMotorStop(forward_right_motor);
            break;

        default:
            break;
    }
    speed_right = forward_right_motor->measure.speed_aps;
    speed_left = forward_left_motor->measure.speed_aps;
    output_left = forward_left_motor->motor_controller.speed_PID.Output;
    output_right = forward_left_motor->motor_controller.speed_PID.Output;
    // 设置反馈数据,主要是imu和yaw的ecd
    forward_feedback_data.new_left_angle  = left_angle->measure.total_angle;
    forward_feedback_data.new_forward_angle = forward_angle->measure.total_angle;

    // 推送消息
    PubPushMessage(forward_pub, (void *)&forward_feedback_data);
    
}



