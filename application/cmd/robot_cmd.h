#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H
//初始化 编码器值
#define LIFT_INIT_ANGLE_RIGHT 151
#define LIFT_INIT_ANGLE_LEFT 181

#define STRETCH_1_INIT_ANGLE_LEFT 127
#define STRETCH_1_INIT_ANGLE_RIGHT 38

#define STRETCH_2_INIT_ANGLE_LEFT 73
#define STRETCH_2_INIT_ANGLE_RIGHT 155

#define HORIZONTAL_INIT 0
//限位 编码器值

#define LIFT_MAX_ANGLE_RIGHT 151
#define LIFT_MIN_ANGLE_RIGHT -21000
#define LIFT_MAX_ANGLE_LEFT 21000
#define LIFT_MIN_ANGLE_LEFT 181

#define STRETCH_1_MIN_ANGLE_RIGHT -23500
#define STRETCH_1_MAX_ANGLE_RIGHT 19000
#define STRETCH_1_MIN_ANGLE_LEFT -19000
#define STRETCH_1_MAX_ANGLE_LEFT 235000

#define STRETCH_2_MAX_ANGLE_RIGHT 14517
#define STRETCH_2_MIN_ANGLE_RIGHT 178
#define STRETCH_2_MAX_ANGLE_LEFT 50
#define STRETCH_2_MIN_ANGLE_LEFT -13811

#define HORIZONTAL_MIN -12016
#define HORIZONTAL_MAX 12500

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

typedef struct
{
    float stretch_left;
    float stretch_right;

    float lift_left;
    float lift_right;

    float gimbal;
    
}initial;//便于统计最初的必要关节角度角度;

typedef struct
{
    int16_t KX;
    int16_t KY;
    int16_t KW;
    int16_t speed_change_flag;

}chassis_speed;//底盘模式取决

typedef struct
{
    int8_t control_mode;

}mode_direction;

typedef struct
{
    //步数
    int8_t step1;
    int8_t step2;
    int8_t step3;
    int8_t step4;
    int8_t step5;
    int8_t step6;
    int8_t step7;
    int8_t step8;
    int8_t step9;
    int8_t step10;

    int8_t now_step;

}auto_mode_step;

typedef struct
{
    //类型
    int8_t type;

    //自动复位解除限制
    int8_t auto_reset_flag;

    //自动模式启停

}auto_mode_Assistant;

#endif // !ROBOT_CMD_H