#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"
#include "dji_motor.h"
#include "encoder.h"
#include "servo_motor.h"
#include "remote_control.h"


//底盘模式
typedef enum {
    CHASSIS_ZERO_FORCE = 2,    
    CHASSIS_WALK = 3,
} chassis_mode_e;

//升降模式
typedef enum{
    LIFT = 2,
    LIFT_STOP = 3,
}lift_mode_e;

//伸出模式设定
typedef enum{
    STRETCH = 2,
    STRETCH_STOP = 3,
    
}stretch_mode_e;

//大pitch模式设定
typedef enum{
    BIG_PITCH_START = 2,
    BIG_PITCH_STOP = 3,
    
}BIG_PITCH_mode_e;

//YAW模式设定
typedef enum{
    YAW_START = 2,
    YAW_STOP = 3,
    
}YAW_mode_e;

//BIG_ROLL模式设定
typedef enum{
    BIG_ROLL_START = 2,
    BIG_ROLL_STOP = 3,
    
}BIG_ROLL_mode_e;

//SMALL_PITCH模式设定
typedef enum{
    SMALL_PITCH_START = 2,
    SMALL_PITCH_STOP = 3,
    
}SMALL_PITCH_mode_e;

//SMALL_ROLL模式设定
typedef enum{
    SMALL_ROLL_START = 2,
    SMALL_ROLL_STOP = 3,
    
}SMALL_ROLL_mode_e;

//GIMBAL模式设定
typedef enum{
    GIMBAL_START = 2,
    GIMBAL_STOP = 3,
    
}GIMBAL_mode_e;




//底盘控制
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
} Chassis_Ctrl_Cmd_s;

//升降控制
typedef struct
{ 
    float left_angle;
    float right_angle;
    lift_mode_e lift_mode;
} Lift_Ctrl_Cmd_s;

//图传舵机控制
typedef struct 
{
    float pitch_angle;
    float yaw_angle;
}Servo_Cmd_s;

//伸出角度控制
typedef struct
{ 
    float left_angle;
    float right_angle;
    stretch_mode_e stretch_mode;
} Stretch_Ctrl_Cmd_s;

//大pitch角度控制
typedef struct 
{
    
    float angle;
    BIG_PITCH_mode_e BIG_PITCH_mode;

}BigPitch_Ctrl_Cmd_s;

//YAW角度控制
typedef struct 
{
    
    float angle;
    YAW_mode_e YAW_mode;

}YAW_Ctrl_Cmd_s;

//BIG_ROLL角度控制
typedef struct 
{
    
    float angle;
    BIG_ROLL_mode_e BIG_ROLL_mode;

}BIG_ROLL_Ctrl_Cmd_s;

//SMALL_PITCH角度控制
typedef struct 
{
    
    float angle;
    SMALL_PITCH_mode_e SMALL_PITCH_mode;

}SMALL_PITCH_Ctrl_Cmd_s;

//SMALL_ROLL角度控制
typedef struct 
{
    
    float speed;
    SMALL_ROLL_mode_e SMALL_ROLL_mode;

}SMALL_ROLL_Ctrl_Cmd_s;

//GIMBAL角度控制
typedef struct 
{
    
    float angle;
    GIMBAL_mode_e GIMBAL_mode;

}GIMBAL_Ctrl_Cmd_s;

typedef struct 
{
    
  //不知道要回传什么，因为图传链路是单向的

}Vision_Joint_Data_Ctrl_Cmd_s;


//UI信息
typedef struct
{ 
    int16_t flag_refresh_ui;
   

} ui_Ctrl_Cmd_s;




//升降回传信息
typedef struct
{
    float now_left_angle;
    float now_right_angle;

    float now_left_current;
    float now_right_current;

    float now_left_speed;
    float now_right_speed;
} Lift_Upload_Data_s;

//伸出的上传信息
typedef struct
{ 
    float now_left_angle;
    float now_right_angle;

    float now_left_current;
    float now_right_current;

    float now_left_speed;
    float now_right_speed;

}Stretch_Upload_Data_s; 

//舵机
typedef struct
{
    /* data */
}Servo_Upload_Data_s;

//Ui
typedef struct
{ 
   int flag_refresh_ui;

}ui_Upload_Data_s; 

//底盘
typedef struct 
{
    /* data */
}Chassis_Upload_Data_s;

//大pitch
typedef struct 
{
    float torque;
    float now_angle;
    float speed;

    float temp;

}BigPitch_Upload_Data_s;

//YAW
typedef struct 
{
    float torque;
    float now_angle;
    float speed;
    
}YAW_Upload_Data_s;

//BIG_ROLL
typedef struct 
{
    float torque;
    float now_angle;
    float speed;
    
}BIG_ROLL_Upload_Data_s;

//SMALL_PITCH
typedef struct 
{
    float torque;
    float now_angle;
    float speed;
    
}SMALL_PITCH_Upload_Data_s;

//SMALL_ROLL
typedef struct 
{
    float now_angle;
    
}SMALL_ROLL_Upload_Data_s;

//GIMBAL
typedef struct 
{
    float now_angle;
    float current;
    float speed;
    
}GIMBAL_Upload_Data_s;

typedef struct 
{
    float vision_big_pitch;
    float vision_yaw;
    float vision_big_roll;
    float vision_small_pitch;
    float vision_small_roll;
    float vision_stretch;//与伸出的左侧是对应的  车上13,293.633295控制器是46,554.873535
    uint8_t custom_controller_comm_recv;//0xff
    uint8_t run_flag;//启动的标志位
    uint8_t air_pump;//气泵的启停
    uint8_t lift;//升降的启停

}Vision_Joint_Data_Upload_Data_s;



typedef struct
{ 
   

    int main_air_flag;//主气泵
    int mine_air_flag;//矿仓气泵
    int32_t big_pitch_temputure;//大pitch的温度
    int Control_mode; //控制模式 0 是自定义控制器 1是自动模式 2是键鼠
    int custom_connect;
    int auto_type;//1中金 2左金 3右金 4自动抬金 5自动复位 6放肚子 7从肚子里面拿出来 8无矿乌龟 9手里有一个矿的乌龟 10 双矿乌龟
    int chassis_speed;//0高速 1中速 2低速
    int control_refresh;

}UI_data_t;


#endif // !ROBOT_DEF_H