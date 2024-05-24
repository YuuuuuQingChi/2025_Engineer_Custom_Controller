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

void Init_Value();
float Limit_Set(float obj, float max, float min);
#endif // !ROBOT_CMD_H