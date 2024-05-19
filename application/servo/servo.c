#include "servo.h"
#include "servo_motor.h"
#include "robot_def.h"
#include "message_center.h"

static ServoInstance *Pitch_Motor,*Yaw_Motor;
static Publisher_t *servo_pub;                          
static Subscriber_t *servo_sub;                        
static Servo_Upload_Data_s servo_feedback_data;
static Servo_Cmd_s servo_cmd_recv;        

void Servo_Init(){
    Servo_Init_Config_s Pitch_config= {
    .Servo_type=Servo180,
    .Servo_Angle_Type=Free_Angle_mode,
    // 使用的定时器类型及通道
    .htim = &htim2,
    .Channel= TIM_CHANNEL_1,
    };
    Servo_Init_Config_s Yaw_config= {
    .Servo_type=Servo360,
    .Servo_Angle_Type=Free_Angle_mode,
    // 使用的定时器类型及通道
    .htim = &htim2,
    .Channel= TIM_CHANNEL_3,
    };

    Pitch_Motor = ServoInit(&Pitch_config);
    Yaw_Motor = ServoInit(&Yaw_config);
    servo_sub = SubRegister("servo_cmd", sizeof(Servo_Cmd_s));
    servo_pub = PubRegister("servo_feed", sizeof(Servo_Upload_Data_s));
    
}

void Servo_Task(){
    SubGetMessage(servo_sub, &servo_cmd_recv);
    
    Servo_Motor_FreeAngle_Set(Pitch_Motor,servo_cmd_recv.pitch_now_angle);
    Servo_Motor_FreeAngle_Set(Yaw_Motor,servo_cmd_recv.yaw_now_angle);
    // 推送消息
    PubPushMessage(servo_pub, (void *)&servo_feedback_data);
}