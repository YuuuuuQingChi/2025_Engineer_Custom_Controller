#include "bsp_can.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "DMmotor.h"

static uint8_t idx = 0;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_MX_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算

float Hex_To_Float(uint32_t *Byte,int num)//十六进制到浮点数
{
	return *((float*)Byte);
}

uint32_t FloatTohex(float HEX)//浮点数到十六进制转换
{
	return *( uint32_t *)&HEX;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


/**
 * @brief 协议，打算把多圈放在外面
 * 
 * @param _instance 
 */
static void dm4310_fbdata(CANInstance *_instance)
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
	measure->pos = uint_to_float(measure->p_int, P_MIN, P_MAX, 16); // 
	measure->vel = uint_to_float(measure->v_int, V_MIN, V_MAX, 12); // 
	measure->tor = uint_to_float(measure->t_int, T_MIN, T_MAX, 12);  // 
	measure->Tmos = (float)(rx_buff[6]);
	measure->Tcoil = (float)(rx_buff[7]);
}
static void dm8006_fbdata(CANInstance *_instance)
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
	measure->pos = uint_to_float(measure->p_int, -50, 50, 16); // 
	measure->vel = uint_to_float(measure->v_int, -45, 45, 12); // 
	measure->tor = uint_to_float(measure->t_int, -20, 20, 12);  // 
	measure->Tmos = (float)(rx_buff[6]);
	measure->Tcoil = (float)(rx_buff[7]);
}
static void dm6006_fbdata(CANInstance *_instance)
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
	measure->pos = uint_to_float(measure->p_int, P_MIN, P_MAX, 16); // 
	measure->vel = uint_to_float(measure->v_int, -45, 45, 12); // 
	measure->tor = uint_to_float(measure->t_int, -12, 12, 12);  // 
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
    memset(motor, 0, sizeof(DMMotorInstance));
    // 注册电机到CAN总线
    config->can_init_config.id                  = motor;  
    if(config->motor_type == DM4310)
    {
        config->can_init_config.can_module_callback = dm4310_fbdata; 
    }
    else if (config->motor_type == DM8006)
    {
        config->can_init_config.can_module_callback = dm8006_fbdata;
    }
    else if (config->motor_type == DM6006)
    {
        config->can_init_config.can_module_callback = dm6006_fbdata;
    }
        
    motor->motor_can_ins                = CANRegister(&config->can_init_config);

    DMMotorStop(motor);
    DWT_GetDeltaT(&motor->measure.feed_cnt);
    dm_motor_instance[idx++] = motor;
    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback     = DMMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 50, // 50ms未收到数据则丢失
        .init_count = 0,
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}


void DMMotorControl()
{
    float vset;
    float pset;
    DMMotorInstance *motor;
    DMMotor_Measure_t *measure;
    Motor_Control_Setting_s *setting;

    for (size_t i = 0; i < idx; ++i)
    {
        motor = dm_motor_instance[i];
        measure = &motor->measure;

        vset = motor->v_ref;
        pset = motor->p_ref;
        uint8_t *vbuf;
        uint8_t *pbuf;
        vbuf = (uint8_t *)& vset;
        pbuf = (uint8_t *)& pset;
        motor->motor_can_ins->tx_buff[0] = *pbuf;
        motor->motor_can_ins->tx_buff[1] = *(pbuf+1);
        motor->motor_can_ins->tx_buff[2] = *(pbuf+2);
        motor->motor_can_ins->tx_buff[3] = *(pbuf+3);
        motor->motor_can_ins->tx_buff[4] = *vbuf;
        motor->motor_can_ins->tx_buff[5] = *(vbuf+1);
        motor->motor_can_ins->tx_buff[6] = *(vbuf+2);
        motor->motor_can_ins->tx_buff[7] = *(vbuf+3);

        

        if ((motor->stop_flag == MOTOR_STOP)||(motor->stop_flag == MOTOR_ENABLED && motor->measure.state == 0))
        { 
            memset(motor->motor_can_ins->tx_buff, 0, sizeof(motor->motor_can_ins->tx_buff));
            motor->motor_can_ins->tx_buff[0] = 0xFF;
            motor->motor_can_ins->tx_buff[1] = 0xFF;
            motor->motor_can_ins->tx_buff[2] = 0xFF;
            motor->motor_can_ins->tx_buff[3] = 0xFF;
            motor->motor_can_ins->tx_buff[4] = 0xFF;
            motor->motor_can_ins->tx_buff[5] = 0xFF;
            motor->motor_can_ins->tx_buff[6] = 0xFF;
            if(motor->stop_flag == MOTOR_STOP )
            {
            motor->motor_can_ins->tx_buff[7] = 0xFD;
            }
            if(motor->stop_flag == MOTOR_ENABLED && (motor->measure.state == 0 || motor->measure.state != 1))
            {
                motor->motor_can_ins->tx_buff[7] = 0xFC;
            }
            
        }

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

void DMMotorSetRef(DMMotorInstance *motor, float vref , float pref)
{
    motor->p_ref = pref;
    motor->v_ref = vref;
}
