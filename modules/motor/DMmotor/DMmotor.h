#ifndef DMmotor_H
#define DMmotor_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_MX_CNT 14 // 最多允许4个DM电机使用多电机指令,挂载在一条总线上

//以下是DM4310的参数，用其他电机需要更改下面参数
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

// 电机回传信息结构体
typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;

	float pos;
	float vel;
	float tor;

	float Tmos;
	float Tcoil;

    uint32_t feed_cnt;
    float dt;
}DMMotor_Measure_t;

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
    float v_ref;
    float p_ref;

    Motor_Working_Type_e stop_flag; // 启停标志

    CANInstance *motor_can_ins;

    DaemonInstance *daemon;
 
    Motor_Type_e motor_type;        // 电机类型
} DMMotorInstance;

/**
 * @brief 初始化DR电机
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

void DMMotorSetRef(DMMotorInstance *motor, float vref , float pref);

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
