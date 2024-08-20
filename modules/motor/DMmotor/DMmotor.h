#ifndef DMmotor_H
#define DMmotor_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_MX_CNT 9 

#define KP_MIN 0
#define KP_MAX 500
#define KD_MIN 0
#define KD_MAX 5
// 电机回传信息结构体
typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	float pos;//弧度制
	float vel;
	float tor;
	float Tmos;
	float Tcoil;
    uint32_t feed_cnt;
    float dt;
}DMMotor_Measure_t;
//电机控制模式选择
typedef enum
{
    MIT = 3,
    PVCtrl,
    VCtrl,
}DMMotor_Control_Mode_e;

typedef struct
{
    DMMotor_Measure_t measure;

    Motor_Control_Setting_s motor_settings;

    float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;   // 速度前馈数据指针,可以通过此指针设置速度前馈值,或LQR等时作为速度状态变量的输入
    float *current_feedforward_ptr; // 电流前馈指针
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;

    DMMotor_Control_Mode_e DMMotor_Control_Mode;//达妙电机控制模式，分为MIT||位置速度||速度
    float PMAX,VMAX,TMAX;//分别是位置，速度，扭矩的最大值设定，不同dm电机的类型，需要不同的值
    float v_ref;//目标速度/最大速度限制/随模式选择而变
    float p_ref;//目标位置
    float t_ref;//目标扭矩/最大扭矩限制/随模式选择而变
    float KP,KD,t_ref;// 位置环P，位置环D,扭矩给定值

    Motor_Working_Type_e stop_flag; // 启停标志
    CANInstance *motor_can_ins;
    DaemonInstance *daemon;
    Motor_Type_e motor_type;        // 电机类型

} DMMotorInstance;

/**
 * @brief 初始化DM电机
 *
 * @param config 电机配置
 * @return DRMotorInstance* 返回实例指针
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置参考值
 * @attention 注意此函数设定的ref是最外层闭环的输入,若要设定内层闭环的值请通过前馈数据指针设置
 *
 * @param motor 要设置的电机
 * @param ref 设定值
 */

void DMMotor_PV_SetRef(DMMotorInstance *motor, float vref , float pref);

/**
 * @brief 为所有DM电机计算pid/反转/模式控制,并通过bspcan发送电流值(发送CAN报文)
 *
 */
void DMMotorControl();

/**
 * @brief 停止DM电机,之后电机不会响应任何指令
 *
 * @param motor
 */
void DMMotorStop(DMMotorInstance *motor);

/**
 * @brief 启动DM电机
 *
 * @param motor
 */

void DMMotorEnable(DMMotorInstance *motor);
#endif // DMmotor_H
