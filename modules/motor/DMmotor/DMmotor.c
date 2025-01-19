/**
 * @file DMmotor.c
 * @author YuuuuQingChi
 * @brief DM电机协议
 * @version 1.1
 * @date 2024-07-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "bsp_can.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "DMmotor.h"

static uint8_t idx = 0;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_MX_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算
static uint8_t DM_ENABLE[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static uint8_t DM_STOP[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static uint8_t DM_CLEAR_ERROR[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};

float Hex_To_Float(uint32_t *Byte,int num)//十六进制到浮点数
{
	return *((float*)Byte);
}
uint32_t FloatTohex(float HEX)//浮点数到十六进制转换
{
	return *( uint32_t *)&HEX;
}
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


/**
 * @brief 接收帧解析
 * @note 其中的motor->p/v/tmax为电机自带属性，会根据你初始化电机类型选择而自动赋值，不要自行更改
 * @param _instance 
 */
static void DM_FBdata(CANInstance *_instance)
{
    DMMotorInstance *motor = (DMMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    DMMotor_Measure_t *measure = &motor->measure;
    uint8_t *rx_buff = _instance->rx_buff;
    DaemonReload(motor->daemon); // 喂狗
    measure->dt = DWT_GetDeltaT(&measure->feed_cnt);
	measure->id = (rx_buff[0])&0xFF;
	measure->state = (rx_buff[0])>>4;
	measure->p_int=(rx_buff[1]<<8)|rx_buff[2];
	measure->v_int=(rx_buff[3]<<4)|(rx_buff[4]>>4);
	measure->t_int=((rx_buff[4]&0xF)<<8)|rx_buff[5];
	measure->pos = uint_to_float(measure->p_int, (-motor->PMAX), motor->PMAX, 16);
	measure->vel = uint_to_float(measure->v_int, (-motor->VMAX), motor->VMAX, 12); 
	measure->tor = uint_to_float(measure->t_int, (-motor->TMAX), motor->TMAX, 12);  
	measure->Tmos = (float)(rx_buff[6]);
	measure->Tcoil = (float)(rx_buff[7]);
}

/**
 * @brief 丢失处理函数  
 * 
 * @param motor_ptr 
 */
static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    LOGWARNING("[DMMotor] motor lost, id: %d", motor->motor_can_ins->tx_id);
}


// 电机初始化,返回一个电机实例
// 只打算用速度模式
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    //清空数据
    memset(motor, 0, sizeof(DMMotorInstance));
    // 注册电机到CAN总线
    config->can_init_config.id                  = motor;  
    /**
     * @todo后续如果有其他类型的电机，只需添加到这里即可
     * 
     */
    switch (config->motor_type)
    {
    case DM4310:
        motor->PMAX = 12.5f,motor->VMAX = 30.0f,motor->TMAX = 10.0f;
        break;
    case DM6006:
        motor->PMAX = 12.5f,motor->VMAX = 45.0f,motor->TMAX = 12.0f;
        break;
    case DM8006:
        motor->PMAX = 50.0f,motor->VMAX = 45.0f,motor->TMAX = 20.0f;
        break;
    default:
        while (1)
        LOGWARNING("[DMMotor]Please ensure that all motors are initialized with the selected model");
        break;
    }
    
    config->can_init_config.can_module_callback = DM_FBdata;
    motor->motor_can_ins                = CANRegister(&config->can_init_config);

    DMMotorStop(motor);
    DWT_GetDeltaT(&motor->measure.feed_cnt);
    dm_motor_instance[idx++] = motor;
    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback     = DMMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 5, // 50ms未收到数据则丢失
        .init_count = 0,
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}


void DMMotorControl()
{   
    //主要为了提高代码可读性
    float vset;
    float pset;
    float tset;
    DMMotorInstance *motor;
    //DMMotor_Measure_t *measure;
    //Motor_Control_Setting_s *setting;

    for (size_t i = 0; i < idx; ++i)
    {
        motor = dm_motor_instance[i];
        // measure = &motor->measure;
        vset = motor->v_ref;
        pset = motor->p_ref;
        tset = motor->t_ref;
        uint8_t *vbuf;
        uint8_t *pbuf;
        vbuf = (uint8_t *)& vset;
        pbuf = (uint8_t *)& pset;
        if(motor->DMMotor_Control_Mode == PVCtrl)
        {
            motor->motor_can_ins->tx_buff[0] = *pbuf;
            motor->motor_can_ins->tx_buff[1] = *(pbuf+1);
            motor->motor_can_ins->tx_buff[2] = *(pbuf+2);
            motor->motor_can_ins->tx_buff[3] = *(pbuf+3);
            motor->motor_can_ins->tx_buff[4] = *vbuf;
            motor->motor_can_ins->tx_buff[5] = *(vbuf+1);
            motor->motor_can_ins->tx_buff[6] = *(vbuf+2);
            motor->motor_can_ins->tx_buff[7] = *(vbuf+3);
        }
        else if (motor->DMMotor_Control_Mode == VCtrl)
        {
            //速度模式下要修改发送长度
            motor->motor_can_ins->txconf.DataLength = FDCAN_DLC_BYTES_4;
            motor->motor_can_ins->tx_buff[0] = *vbuf;
            motor->motor_can_ins->tx_buff[1] = *(vbuf+1);
            motor->motor_can_ins->tx_buff[2] = *(vbuf+2);
            motor->motor_can_ins->tx_buff[3] = *(vbuf+3);
        }
        else if (motor->DMMotor_Control_Mode == MIT)
        {
            uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
            pos_tmp = float_to_uint(pset, (-motor->PMAX), motor->PMAX, 16);
            vel_tmp = float_to_uint(vset, (-motor->VMAX), motor->VMAX, 12);
            kp_tmp = float_to_uint(motor->KP, KP_MIN, KP_MAX, 12);
            kd_tmp = float_to_uint(motor->KD, KD_MIN, KD_MAX, 12);
            tor_tmp = float_to_uint(tset,(-motor->TMAX ), motor->TMAX, 12);
            motor->motor_can_ins->tx_buff[0] = (pos_tmp >> 8);
            motor->motor_can_ins->tx_buff[1] = pos_tmp;
            motor->motor_can_ins->tx_buff[2] = (vel_tmp >> 4);
            motor->motor_can_ins->tx_buff[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
            motor->motor_can_ins->tx_buff[4] = kp_tmp;
            motor->motor_can_ins->tx_buff[5] = (kd_tmp >> 4);
            motor->motor_can_ins->tx_buff[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
            motor->motor_can_ins->tx_buff[7] = tor_tmp;
        }

        if(motor->stop_flag == MOTOR_ENABLED && motor->measure.state != 0 && motor->measure.state != 1)memcpy(motor->motor_can_ins->tx_buff, DM_CLEAR_ERROR, sizeof(DM_CLEAR_ERROR));
        if(motor->stop_flag == MOTOR_STOP)memcpy(motor->motor_can_ins->tx_buff, DM_STOP, sizeof(DM_STOP));
        if(motor->stop_flag == MOTOR_ENABLED && motor->measure.state == 0)memcpy(motor->motor_can_ins->tx_buff, DM_ENABLE, sizeof(DM_ENABLE));
        //发送
        CANTransmit(motor->motor_can_ins, 1);
    }        
}
void DMMotorStop(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENABLED;
}

/**
 * @brief 速度和位置给定，位速模式下两个参数都要给，速度模式下可以不用管pref
 * 
 * @param motor 
 * @param vref 速度给定值
 * @param pref 位置给定值
 */
void DMMotor_PV_SetRef(DMMotorInstance *motor, float vref , float pref)
{
    motor->p_ref = pref;
    motor->v_ref = vref;
}

/**
 * @brief 由于mit模式自己多出了3个参数，所以多出一个函数，便于MIT控制
 * 
 * @param motor 
 * @param KP 位置环KP
 * @param KD 位置环KD
 * @param t_ref 扭矩给定值
 */
void DMMotor_MIT_SetRef(DMMotorInstance *motor, float KP , float KD , float t_ref)
{
    motor->KP = KP;
    motor->KD = KD;
    motor->t_ref = t_ref;
}
