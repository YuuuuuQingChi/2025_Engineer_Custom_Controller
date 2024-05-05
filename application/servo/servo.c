#include "servo.h"
#include "servo_motor.h"
#include "robot_def.h"
#include "message_center.h"

static ServoInstance *Pitch_Motor,*Yaw_Motor;
static Publisher_t *servo_pub;                          // 升降应用消息发布者(升降反馈给cmd)
static Subscriber_t *servo_sub;                         // cmd控制消息订阅者
static Servo_Upload_Data_s servo_feedback_data; // 回传给cmd的升降状态信息
static Servo_Cmd_s servo_cmd_recv;         // 来自cmd的控制信息

void Servo_Init(){
    Servo_Init_Config_s Pitch_config= {
    .Servo_type=Servo360,
    .Servo_Angle_Type=Free_Angle_mode,
    // 使用的定时器类型及通道
    .htim=&htim1,
    /*Channel值设定
     *TIM_CHANNEL_1
     *TIM_CHANNEL_2
     *TIM_CHANNEL_3
     *TIM_CHANNEL_4
     *TIM_CHANNEL_ALL
     */
    .Channel=TIM_CHANNEL_1
    };

    Pitch_Motor=ServoInit(&Pitch_config);
    servo_pub = PubRegister("servo_feed", sizeof(Servo_Upload_Data_s));
    servo_sub = SubRegister("servo_cmd", sizeof(Servo_Cmd_s));
}

void Servo_Task(){
    SubGetMessage(servo_sub, &servo_cmd_recv);
    
    // 推送消息
    PubPushMessage(servo_pub, (void *)&servo_feedback_data);
}