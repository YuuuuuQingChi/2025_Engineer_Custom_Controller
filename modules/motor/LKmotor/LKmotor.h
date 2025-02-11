#ifndef LKmotor_H
#define LKmotor_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#include "stdbool.h"
#define LK_MOTOR_MX_CNT        4 // 最多允许4个LK电机使用多电机指令,挂载在一条总线上

#define I_MIN                  -2000
#define I_MAX                  2000
#define CURRENT_SMOOTH_COEF    0.9f
#define SPEED_SMOOTH_COEF      0.85f
#define REDUCTION_RATIO_DRIVEN 1
// #define ECD_ANGLE_COEF_LK (360.0f / 65536.0f)
#define CURRENT_TORQUE_COEF_LK 0.003645f // 电流设定值转换成扭矩的系数,算出来的设定值除以这个系数就是扭矩值

typedef struct
{

    int8_t command_data;
    int8_t temperature; // 温度,C°
    int16_t iq;
    int16_t raw_speed;
    uint16_t raw_encoder_angle;

    uint16_t last_raw_encoder_angle; // 上一次读取的编码器值
    int32_t total_round;             // 总圈数
    float resolution;                // 分辨率(关于电流的)
    float angle_single_round;        // 单圈角度
    uint16_t max_ecd;                // 编码器数值范围
    float torque_constant;
    uint32_t offset;
    bool is_reserved;

    float total_radian; // 总角度rad
    float speed_rpm;    // speed rad/s
    float torque;

    float feed_dt;
    uint32_t feed_dwt_cnt;

} LKMotor_Measure_t;

typedef struct
{
    LKMotor_Measure_t measure;

    Motor_Control_Setting_s motor_settings;

    float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;   // 速度前馈数据指针,可以通过此指针设置速度前馈值,或LQR等时作为速度状态变量的输入
    float *current_feedforward_ptr; // 电流前馈指针
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float pid_ref;

    Motor_Working_Type_e stop_flag; // 启停标志

    CANInstance *motor_can_ins;

    DaemonInstance *daemon;

    Motor_Type_e motor_type; // 电机类型
} LKMotorInstance;

/**
 * @brief 初始化LK电机
 *
 * @param config 电机配置
 * @return LKMotorInstance* 返回实例指针
 */
LKMotorInstance *LKMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置参考值
 * @attention 注意此函数设定的ref是最外层闭环的输入,若要设定内层闭环的值请通过前馈数据指针设置
 *
 * @param motor 要设置的电机
 * @param ref 设定值
 */
void LKMotorSetRef(LKMotorInstance *motor, float ref);

/**
 * @brief 为所有LK电机计算pid/反转/模式控制,并通过bspcan发送电流值(发送CAN报文)
 *
 */
void LKMotorControl();

/**
 * @brief 停止LK电机,之后电机不会响应任何指令
 *
 * @param motor
 */
void LKMotorStop(LKMotorInstance *motor);

/**
 * @brief 启动LK电机
 *
 * @param motor
 */
void LKMotorEnable(LKMotorInstance *motor);

uint8_t LKMotorIsOnline(LKMotorInstance *motor);
void LKMotorSetSpeed(LKMotorInstance *motor, int32_t speed);
#endif // LKmotor_H
