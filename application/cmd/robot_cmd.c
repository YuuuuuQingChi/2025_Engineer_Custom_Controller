// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "ramp.h"
#include <UI.h>
//  module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "buzzer.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "math.h"
#include "vision_line.h"

/* cmd应用包含的模块实例指针和交互信息存储*/

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回

static Publisher_t *stretch_cmd_pub;                    // 二级控制消息发布者
static Subscriber_t *stretch_feed_sub;                  // 二级反馈信息订阅者
static Stretch_Ctrl_Cmd_s stretch_cmd_send;      // 传递给二级的控制信息
static Stretch_Upload_Data_s stretch_fetch_data; // 从二级获取的反馈信息

static Publisher_t *BigPitch_cmd_pub;
static Subscriber_t *BigPitch_feed_sub;
static BigPitch_Ctrl_Cmd_s BigPitch_cmd_send;
static BigPitch_Upload_Data_s BigPitch_fetch_data;

static Publisher_t *YAW_cmd_pub;
static Subscriber_t *YAW_feed_sub;
static YAW_Ctrl_Cmd_s YAW_cmd_send;
static YAW_Upload_Data_s YAW_fetch_data;

static Publisher_t *BIG_ROLL_cmd_pub;
static Subscriber_t *BIG_ROLL_feed_sub;
static BIG_ROLL_Ctrl_Cmd_s BIG_ROLL_cmd_send;
static BIG_ROLL_Upload_Data_s BIG_ROLL_fetch_data;

static Publisher_t *SMALL_PITCH_cmd_pub;
static Subscriber_t *SMALL_PITCH_feed_sub;
static SMALL_PITCH_Ctrl_Cmd_s SMALL_PITCH_cmd_send;
static SMALL_PITCH_Upload_Data_s SMALL_PITCH_fetch_data;

static Publisher_t *SMALL_ROLL_cmd_pub;
static Subscriber_t *SMALL_ROLL_feed_sub;
static SMALL_ROLL_Ctrl_Cmd_s SMALL_ROLL_cmd_send;
static SMALL_ROLL_Upload_Data_s SMALL_ROLL_fetch_data;

static Publisher_t *lift_cmd_pub;
static Subscriber_t *lift_feed_sub;
static Lift_Ctrl_Cmd_s lift_cmd_send;
static Lift_Upload_Data_s lift_fetch_data;

static Publisher_t *servo_cmd_pub;
static Subscriber_t *servo_feed_sub;
static Servo_Cmd_s servo_cmd_send;
static Servo_Upload_Data_s servo_fetch_data;

static Publisher_t *ui_cmd_pub;
static UI_data_t ui_cmd_send;

static Publisher_t *vision_joint_data_pub;
static Subscriber_t *vision_joint_data_sub;
static Vision_Joint_Data_Upload_Data_s Vision_Joint_Data_fetch_data; 
static Vision_Joint_Data_Ctrl_Cmd_s Vision_Joint_Data_cmd_send;  

static Publisher_t *GIMBAL_cmd_pub;
static Subscriber_t *GIMBAL_feed_sub;
static GIMBAL_Upload_Data_s GIMBAL_fetch_data; 
static GIMBAL_Ctrl_Cmd_s GIMBAL_Data_cmd_send;  

static initial initial_angle;
static chassis_speed chassis_speed_scale; //2是慢速 3是中速 4是高速;
static auto_mode_step gold_auto[3] , silver_auto , pull_gold ,  auto_reset , receive_mine , pull_inter_mine ,turtle[3];
static auto_mode_Assistant auto_Assistant;//type: 11是停止（且默认为11） 2是左金,3是中金,4是右金,5是银矿,6是自动抬金矿，7是自动复位 , 8是放回肚子 ， 9是从肚子拿出来
static mode_direction mode;//2是自定义控制器。3是自动 。4是键鼠

extern int mouse_count_r;
uint8_t big_pitch_stall_flag = 3;
uint8_t gimbal_stall_flag = 3;
float direction;
float direction1;


USARTInstance *vision_usart;

/**
 * @brief 图传链路信息传递函数
 * @todo 后续加入键鼠数据传递
 * 
 */
static void RobotCMDInit_VisionLine()
{
    USART_Init_Config_s vision_usart_conf = {
        .module_callback = vision_recv_callback,
        .recv_buff_size  = 255,
        .usart_handle    = &huart1, 
        .checkout_callback = NULL,
        
    };
   
    vision_usart       = USARTRegister(&vision_usart_conf);// 图传串口
}

static void initial_calc()
{
    initial_angle.stretch_left = stretch_fetch_data.now_left_angle;
    initial_angle.stretch_right = stretch_fetch_data.now_right_angle;
    initial_angle.lift_left = lift_fetch_data.now_left_angle;
    initial_angle.lift_right = lift_fetch_data.now_right_angle;
    initial_angle.gimbal = GIMBAL_fetch_data.now_angle;

    chassis_speed_scale.speed_change_flag = 3;//默认快速
    mode.control_mode = 2;//默认自定义控制器

    servo_cmd_send.yaw_angle = 95;
    servo_cmd_send.pitch_angle = 140;

    auto_Assistant.type = 21;//默认什么也不选

    auto_Assistant.auto_reset_flag = 3;//默认开

    

}


/**
 * @brief CMD初始化/
 *
 */
void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

    lift_cmd_pub            = PubRegister("lift_cmd", sizeof(Lift_Ctrl_Cmd_s));
    lift_feed_sub           = SubRegister("lift_feed", sizeof(Lift_Upload_Data_s));

    stretch_cmd_pub  = PubRegister("stretch_cmd", sizeof(Stretch_Ctrl_Cmd_s));
    stretch_feed_sub = SubRegister("stretch_feed", sizeof(Stretch_Upload_Data_s));

    chassis_cmd_pub         = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub        = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    BigPitch_cmd_pub        = PubRegister("BIG_PITCH_cmd", sizeof(BigPitch_Ctrl_Cmd_s));
    BigPitch_feed_sub       = SubRegister("BIG_PITCH_feed", sizeof(BigPitch_Upload_Data_s));

    YAW_cmd_pub        = PubRegister("YAW_cmd", sizeof(YAW_Ctrl_Cmd_s));
    YAW_feed_sub       = SubRegister("YAW_feed", sizeof(YAW_Upload_Data_s));

    BIG_ROLL_cmd_pub        = PubRegister("BIG_ROLL_cmd", sizeof(BIG_ROLL_Ctrl_Cmd_s));
    BIG_ROLL_feed_sub       = SubRegister("BIG_ROLL_feed", sizeof(BIG_ROLL_Upload_Data_s));

    SMALL_PITCH_cmd_pub        = PubRegister("SMALL_PITCH_cmd", sizeof(SMALL_PITCH_Ctrl_Cmd_s));
    SMALL_PITCH_feed_sub       = SubRegister("SMALL_PITCH_feed", sizeof(SMALL_PITCH_Upload_Data_s));

    SMALL_ROLL_cmd_pub        = PubRegister("SMALL_ROLL_cmd", sizeof(SMALL_ROLL_Ctrl_Cmd_s));
    SMALL_ROLL_feed_sub       = SubRegister("SMALL_ROLL_feed", sizeof(SMALL_ROLL_Upload_Data_s));

    servo_cmd_pub  = PubRegister("servo_cmd", sizeof(Servo_Cmd_s));
    servo_feed_sub = SubRegister("servo_feed", sizeof(Servo_Upload_Data_s));

    ui_cmd_pub  = PubRegister("UI", sizeof(UI_data_t));
    

    vision_joint_data_pub  = PubRegister("vision_joint_data_cmd", sizeof(Vision_Joint_Data_Ctrl_Cmd_s));
    vision_joint_data_sub = SubRegister("vision_joint_data_feed", sizeof(Vision_Joint_Data_Upload_Data_s));

    GIMBAL_cmd_pub  = PubRegister("GIMBAL_cmd", sizeof(GIMBAL_Ctrl_Cmd_s));
    GIMBAL_feed_sub = SubRegister("GIMBAL_feed", sizeof(GIMBAL_Upload_Data_s));

    RobotCMDInit_VisionLine();
    initial_calc();
    
}

/**
 * @brief 限位函数，目前除了伸出和升降不太绝对成功适用外，其他关节100%都可以的
 * @param 参考的关节角度值
 * @param 最大角度值
 * @param 最小角度
 */
float Limit_Set(float reference_value, float max, float min)
{   
    
    if (reference_value >= max) {
        return max;
    }
    else if (reference_value <= min) {
        return min;
    }
    else{
        return reference_value;
    }
}

/**
 * @brief Stall
 * @param 参考的关节当前扭矩即为当时反馈扭矩
 * @param 参考的关节当前角度即为当时反馈角度
 * @param 当不进入堵转情况时本应该的控制量
 * 
 */
 
static float Stall(float reference_current,float target_angle,float current_angle,float max_current,float current_speeed,float min_speed)
{
    if((reference_current > max_current || reference_current < - max_current) && (current_speeed < min_speed || current_speeed > -min_speed))
    {
        return target_angle;
    }
    else
    {
        return current_angle;
    }
}

/**
 * @brief 限位函数
 *  
 */
void cmd_value_limit()
{

    if (auto_Assistant.auto_reset_flag == 3)
    {
        /* code */
        lift_cmd_send.left_angle   = Limit_Set(lift_cmd_send.left_angle, initial_angle.lift_left , initial_angle.lift_left - 15469.013979);
        lift_cmd_send.right_angle  = Limit_Set(lift_cmd_send.right_angle, initial_angle.lift_right + 15469.013979 , initial_angle.lift_right);

        stretch_cmd_send.left_angle   = Limit_Set(stretch_cmd_send.left_angle, initial_angle.stretch_left,initial_angle.stretch_left - 9744.35205);
        stretch_cmd_send.right_angle = Limit_Set(stretch_cmd_send.right_angle, 9744.35205 + initial_angle.stretch_right,initial_angle.stretch_right);
    }
   
    BIG_ROLL_cmd_send.angle = Limit_Set(BIG_ROLL_cmd_send.angle,1.927786,-1.873998);

    BigPitch_cmd_send.angle = Limit_Set(BigPitch_cmd_send.angle,3.36995316,-0.502784729);

    YAW_cmd_send.angle = Limit_Set(YAW_cmd_send.angle,1.721027,-1.78664);

    SMALL_PITCH_cmd_send.angle = Limit_Set(SMALL_PITCH_cmd_send.angle,1.91825008,-2.03803349);

    VAL_LIMIT(servo_cmd_send.yaw_angle,50,130);
    VAL_LIMIT(servo_cmd_send.pitch_angle,100,170);

}

/**
 * @brief 堵转检测
 * @todo 后续加入升降和伸出的堵转检测，目前给的限制值都是按照额定给的，后续要具体测量
 * 
 */

void Stall_Detection()
{
    //SMALL_PITCH_cmd_send.angle = Stall(SMALL_PITCH_fetch_data.torque,SMALL_PITCH_fetch_data.now_angle,SMALL_PITCH_cmd_send.angle,3.5,SMALL_PITCH_fetch_data.speed,2);
    BIG_ROLL_cmd_send.angle = Stall(BIG_ROLL_fetch_data.torque,BIG_ROLL_fetch_data.now_angle,BIG_ROLL_cmd_send.angle,3.5,BIG_ROLL_fetch_data.speed,1.1);
    YAW_cmd_send.angle = Stall(YAW_fetch_data.torque,YAW_fetch_data.now_angle,YAW_cmd_send.angle,7,YAW_fetch_data.speed,1);
   //GIMBAL_Data_cmd_send.angle = Stall(GIMBAL_fetch_data.current,GIMBAL_fetch_data.now_angle,GIMBAL_Data_cmd_send.angle,11000,GIMBAL_fetch_data.speed,10);
    
    if(big_pitch_stall_flag == 3){
        if((BigPitch_fetch_data.torque > 9.2 || BigPitch_fetch_data.torque < - 9.2) && (BigPitch_fetch_data.speed < 0.0149901428 || BigPitch_fetch_data.speed > -0.0149901428))
        {
            direction = BigPitch_cmd_send.angle - BigPitch_fetch_data.now_angle;//方向得知
            big_pitch_stall_flag = 2;

        }
        else if ((BigPitch_fetch_data.torque > 12 || BigPitch_fetch_data.torque < - 12))
        {
            direction = BigPitch_cmd_send.angle - BigPitch_fetch_data.now_angle;//方向得知
            big_pitch_stall_flag = 2;

        }
        else
        {
            big_pitch_stall_flag = 3;
        }
    }
    if(big_pitch_stall_flag == 2)
    {
        //if(direction * (BigPitch_cmd_send.angle - BigPitch_fetch_data.now_angle) <= 0)
        //{
           // BigPitch_cmd_send.angle = BigPitch_cmd_send.angle;
            //if(fabs(BigPitch_fetch_data.speed) > 0.0209901428) big_pitch_stall_flag=3;
            //else big_pitch_stall_flag=2;
        //}else if (direction * (BigPitch_cmd_send.angle - BigPitch_fetch_data.now_angle) > 0)
        //{
          //  BigPitch_cmd_send.angle = BigPitch_fetch_data.now_angle;  
            //big_pitch_stall_flag=2;
        //}
         float limit_angle = 0.05;
        //  if(direction * (BigPitch_cmd_send.angle - BigPitch_fetch_data.now_angle) <= 0)
        // {
        //     BigPitch_cmd_send.angle = BigPitch_cmd_send.angle;
        //     big_pitch_stall_flag = 3;
        // }
        VAL_LIMIT(BigPitch_cmd_send.angle, BigPitch_fetch_data.now_angle-limit_angle, BigPitch_fetch_data.now_angle+limit_angle);
            if(fabs(BigPitch_fetch_data.speed) > 0.0209901428) big_pitch_stall_flag=3;
    }


    if(gimbal_stall_flag == 3){
        if((GIMBAL_fetch_data.current > 3700 || GIMBAL_fetch_data.current < - 3700) && (GIMBAL_fetch_data.speed < 4 && GIMBAL_fetch_data.speed > -4))
        {
            direction = GIMBAL_Data_cmd_send.angle - GIMBAL_fetch_data.now_angle;//方向得知
            gimbal_stall_flag = 2;

        }
        // else if ((GIMBAL_fetch_data.current > 7200 || GIMBAL_fetch_data.current < - 7200))
        // {
        //     direction = GIMBAL_Data_cmd_send.angle - GIMBAL_fetch_data.now_angle;//方向得知
        //     gimbal_stall_flag = 2;

        // }
        else
        {
            gimbal_stall_flag = 3;
        }
    }
    if(gimbal_stall_flag == 2)
    {
        if(direction * (GIMBAL_Data_cmd_send.angle - GIMBAL_fetch_data.now_angle) < 0)
        {
            GIMBAL_Data_cmd_send.angle = GIMBAL_Data_cmd_send.angle;
            if(fabs(GIMBAL_fetch_data.speed) > 2) gimbal_stall_flag=3;
            else gimbal_stall_flag = 2;
        }else if (direction * (GIMBAL_Data_cmd_send.angle - GIMBAL_fetch_data.now_angle) > 0)
        {
            GIMBAL_Data_cmd_send.angle = GIMBAL_fetch_data.now_angle;  
            gimbal_stall_flag = 2;
        }
       
         // if(direction * (BigPitch_cmd_send.angle - BigPitch_fetch_data.now_angle) <= 0)
        // {
        //     BigPitch_cmd_send.angle = BigPitch_cmd_send.angle;
        //     big_pitch_stall_flag = 3;
        // }
        //VAL_LIMIT(BigPitch_cmd_send.angle, BigPitch_fetch_data.now_angle-limit_angle, BigPitch_fetch_data.now_angle+limit_angle);
        //if(fabs(BigPitch_fetch_data.speed) > 0.0209901428) big_pitch_stall_flag=3;
    }

}

/**
 * @brief 遥控器消抖函数
 *
 */
int is_range(int a)
{
    if ((a > 2) || (a < -2)) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */

static void RemoteControlSet()
{
    // 左侧开关状态[上],右侧开关状态[上]
    if ((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left)) {
        chassis_cmd_send.vx = rc_data[TEMP].rc.rocker_l_ * 25;
        chassis_cmd_send.vy = rc_data[TEMP].rc.rocker_l1 * 16;
        chassis_cmd_send.wz = rc_data[TEMP].rc.rocker_r_ * 8;

        if (is_range(rc_data[TEMP].rc.rocker_r1)) {
            lift_cmd_send.left_angle = rc_data[TEMP].rc.rocker_r1 / 10.0 + lift_fetch_data.now_left_angle;
            lift_cmd_send.right_angle = -rc_data[TEMP].rc.rocker_r1 / 10.0 + lift_fetch_data.now_right_angle;
        }
    }

    // 左侧开关状态[上],右侧开关状态[中]
    if ((switch_is_mid(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left)) {
        // 伸出 左垂直
        if (is_range(rc_data[TEMP].rc.rocker_l1)) {
            stretch_cmd_send.left_angle = stretch_fetch_data.now_left_angle + rc_data[TEMP].rc.rocker_l1 / 10.0;
            stretch_cmd_send.right_angle =  stretch_fetch_data.now_right_angle - rc_data[TEMP].rc.rocker_l1 / 10.0;
        }
       
            servo_cmd_send.pitch_angle -= 0.04 * rc_data[TEMP].rc.rocker_r1 / 660.0;
            servo_cmd_send.yaw_angle   -= 0.04 * rc_data[TEMP].rc.rocker_r_ / 660.0;
        
        if(is_range(rc_data[TEMP].rc.rocker_l_))
        {
            GIMBAL_Data_cmd_send.angle = GIMBAL_fetch_data.now_angle + rc_data[TEMP].rc.rocker_l_ / 660.0 * 300;
        }
       

    // 左侧开关状态[上],右侧开关状态[下]
    }
    if ((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left)) {

        //大ROLL左水平
        if (is_range(rc_data[TEMP].rc.rocker_l_)){
            BIG_ROLL_cmd_send.angle = BIG_ROLL_fetch_data.now_angle + rc_data[TEMP].rc.rocker_l_ / 660.0 * 0.5;
        }
        //YAW
        if (is_range(rc_data[TEMP].rc.rocker_r_)){
            YAW_cmd_send.angle = YAW_fetch_data.now_angle - rc_data[TEMP].rc.rocker_r_ / 660.0 * 0.3;
        }
        //BIG_PITCH
        if (is_range(rc_data[TEMP].rc.rocker_r1)){
            BigPitch_cmd_send.angle  = BigPitch_fetch_data.now_angle - rc_data[TEMP].rc.rocker_r1 / 660.0 * 0.3; 
        }
        //SMALL_PITCH
        if (is_range(rc_data[TEMP].rc.rocker_l1)) {
            SMALL_PITCH_cmd_send.angle = SMALL_PITCH_fetch_data.now_angle - rc_data[TEMP].rc.rocker_l1 / 660.0 * 0.5;
        }
        //SMALL_ROLL
        if (is_range(rc_data[TEMP].rc.dial)) {

            SMALL_ROLL_cmd_send.speed = rc_data[TEMP].rc.dial > 0? 8000 : -8000;
        }
        else if (!is_range(rc_data[TEMP].rc.dial))
        {
            SMALL_ROLL_cmd_send.speed = 0;
        }
    }
} 

/**
 * @brief 启停函数
 * 
 */
static void Emergency_Handling()
{
// 双下
    if ((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left)) {
        lift_cmd_send.lift_mode                     = LIFT_STOP;
        stretch_cmd_send.stretch_mode               = STRETCH_STOP;
        chassis_cmd_send.chassis_mode               = CHASSIS_ZERO_FORCE;
        BigPitch_cmd_send.BIG_PITCH_mode            = BIG_PITCH_STOP;
        YAW_cmd_send.YAW_mode                       = YAW_STOP;
        BIG_ROLL_cmd_send.BIG_ROLL_mode             = BIG_ROLL_STOP;
        SMALL_PITCH_cmd_send.SMALL_PITCH_mode       = SMALL_PITCH_STOP;
        SMALL_ROLL_cmd_send.SMALL_ROLL_mode         = SMALL_ROLL_STOP;
        GIMBAL_Data_cmd_send.angle                  = GIMBAL_STOP;

        if (is_range(rc_data[TEMP].rc.dial)) {
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
    } else if((rc_data[TEMP].rc.switch_right != 0 && rc_data[TEMP].rc.switch_left !=0) && !((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left))) {
        chassis_cmd_send.chassis_mode               = CHASSIS_WALK;
        lift_cmd_send.lift_mode                     = LIFT;
        stretch_cmd_send.stretch_mode               = STRETCH;
         BigPitch_cmd_send.BIG_PITCH_mode           = BIG_PITCH_START;
        YAW_cmd_send.YAW_mode                       = YAW_START;
        BIG_ROLL_cmd_send.BIG_ROLL_mode             = BIG_ROLL_START;
        SMALL_PITCH_cmd_send.SMALL_PITCH_mode       = SMALL_PITCH_START;
        SMALL_ROLL_cmd_send.SMALL_ROLL_mode         = SMALL_ROLL_START;
        GIMBAL_Data_cmd_send.GIMBAL_mode            = GIMBAL_START;
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void Chassis_MouseKeySet()
{

    if(rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].c)
    {
        chassis_speed_scale.speed_change_flag = 3;
    }
    else if(rc_data[TEMP].key[KEY_PRESS].c)
    {
        chassis_speed_scale.speed_change_flag = 4;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].c)
    {
        chassis_speed_scale.speed_change_flag = 2;
    }

    if (chassis_speed_scale.speed_change_flag == 2)
    {
        chassis_speed_scale.KW = 5.5;
        chassis_speed_scale.KX = 5.5;
        chassis_speed_scale.KY = 5.5;    

        ui_cmd_send.chassis_speed = 2;
    }
    else if(chassis_speed_scale.speed_change_flag == 3)
    {
        chassis_speed_scale.KW = 7;
        chassis_speed_scale.KX = 14;
        chassis_speed_scale.KY = 10;
        ui_cmd_send.chassis_speed = 1;
    }
    else if(chassis_speed_scale.speed_change_flag == 4)
    {
        chassis_speed_scale.KW = 12;
        chassis_speed_scale.KX = 32;
        chassis_speed_scale.KY = 20;
        ui_cmd_send.chassis_speed = 0;
    }
  
    if (rc_data[TEMP].key[KEY_PRESS].w || rc_data[TEMP].key[KEY_PRESS].s) {
        chassis_cmd_send.vy = (rc_data[TEMP].key[KEY_PRESS].w * 660 * chassis_speed_scale.KX - rc_data[TEMP].key[KEY_PRESS].s * 660 * chassis_speed_scale.KX) * ramp_calc(&chassis_vy_ramp);
    } 
    else {
        ramp_init(&chassis_vy_ramp, 600);
        if(chassis_cmd_send.vy > 60)
        {
            chassis_cmd_send.vy -= 60;
        }
        else if(chassis_cmd_send.vy < -60)
        {
            chassis_cmd_send.vy += 60;
        }
        else{
            chassis_cmd_send.vy = 0;
        }
    }
    if (rc_data[TEMP].key[KEY_PRESS].a || rc_data[TEMP].key[KEY_PRESS].d) {
        chassis_cmd_send.vx = (rc_data[TEMP].key[KEY_PRESS].d * 660 * chassis_speed_scale.KY - rc_data[TEMP].key[KEY_PRESS].a * 660 * chassis_speed_scale.KY) * ramp_calc(&chassis_vx_ramp);
    } else {
        ramp_init(&chassis_vx_ramp, 700);
        if(chassis_cmd_send.vx > 80)
        {
            chassis_cmd_send.vx -= 80;
        }
        else if(chassis_cmd_send.vx < -80)
        {
            chassis_cmd_send.vx += 80;
        }
        else
        {
            chassis_cmd_send.vx = 0;
        }
    }
    if (rc_data[TEMP].key[KEY_PRESS].q || rc_data[TEMP].key[KEY_PRESS].e) {
        chassis_cmd_send.wz = (-rc_data[TEMP].key[KEY_PRESS].q * 660 * chassis_speed_scale.KW +rc_data[TEMP].key[KEY_PRESS].e * 660 * chassis_speed_scale.KW) * ramp_calc(&chassis_vw_ramp);
    } else {
        ramp_init(&chassis_vw_ramp, ACCLE_RAMP_TIME);
        if(chassis_cmd_send.wz > 120)
        {
            chassis_cmd_send.wz -= 120;  
        }
        else if(chassis_cmd_send.wz < -120)
        {
            chassis_cmd_send.wz += 120;
        }
        else
        {
            chassis_cmd_send.wz =0;
        }
    }
       
  
}

/**
 * @brief 气路控制
 * @attention PC10引脚是大气泵 
 * @attention PB9引脚是大气泵阀
 */
void air_controll()
{

    // 大气泵 启停 应当注意非左上右下模式，会与roll产生干扰
    // ui_cmd_send.main_air_flag为1是开 为2是关

    if(mode.control_mode == 2)
    {
        if ((Vision_Joint_Data_fetch_data.air_pump == 2)) {
            ui_cmd_send.main_air_flag = 1;
        } else if ((Vision_Joint_Data_fetch_data.air_pump == 3)) {
            ui_cmd_send.main_air_flag = 2;
        }
    }
    if ((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left) && mode.control_mode != 2)
    {
        if (rc_data[TEMP].key[KEY_PRESS].v){
            ui_cmd_send.main_air_flag = 1;
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].v){
             ui_cmd_send.main_air_flag = 2;
         }
    }
    if(!((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left)))
    {
        if(rc_data[TEMP].rc.dial > 250){
            ui_cmd_send.main_air_flag = 1;
        }
        else if (rc_data[TEMP].rc.dial < -250)
        {
            ui_cmd_send.main_air_flag = 2;
        }
    }
    else
    {
         if(abs(rc_data[TEMP].rc.dial) > 250){
            ui_cmd_send.control_refresh = 1;
        }else{
            ui_cmd_send.control_refresh = 0;
        }
        
    }

        
    

    if(rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].v)
    {
        ui_cmd_send.mine_air_flag = 1;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].v)
    {
        ui_cmd_send.mine_air_flag = 2;
    }
   
    if (!((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left))) {
        // 大气泵
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, ui_cmd_send.main_air_flag == 1 ? 1 : 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1 ? 0 : 1);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,ui_cmd_send.mine_air_flag == 1? 1 : 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 1 ? 0 : 1);
    }
        
}
/**
 * @brief 自定义控制器的控制
 * 
 */
void Vision_Control()
{
    
        //要加上防断连
        BIG_ROLL_cmd_send.angle = Vision_Joint_Data_fetch_data.vision_big_roll;
        SMALL_PITCH_cmd_send.angle = Vision_Joint_Data_fetch_data.vision_small_pitch;
        YAW_cmd_send.angle = Vision_Joint_Data_fetch_data.vision_yaw;
        BigPitch_cmd_send.angle = Vision_Joint_Data_fetch_data.vision_big_pitch;
        SMALL_ROLL_cmd_send.speed = Vision_Joint_Data_fetch_data.vision_small_roll;
        stretch_cmd_send.left_angle = Vision_Joint_Data_fetch_data.vision_stretch + initial_angle.stretch_left;//加上起初角度
        stretch_cmd_send.right_angle = -Vision_Joint_Data_fetch_data.vision_stretch - initial_angle.stretch_right;
        //升降是按钮控制，加上斜坡函数
        if(Vision_Joint_Data_fetch_data.lift == 2)
    {
        lift_cmd_send.left_angle = lift_fetch_data.now_left_angle + 200 * ramp_calc(&lift_l_ramp);
        lift_cmd_send.right_angle = lift_fetch_data.now_right_angle - 200 * ramp_calc(&lift_r_ramp);
    }
    else if (Vision_Joint_Data_fetch_data.lift == 3)
    {
        lift_cmd_send.left_angle = lift_fetch_data.now_left_angle - 200 * ramp_calc(&lift_l_ramp);
        lift_cmd_send.right_angle = lift_fetch_data.now_right_angle + 200 * ramp_calc(&lift_r_ramp);
    }
    else{
        ramp_init(&lift_l_ramp, 800);
        ramp_init(&lift_r_ramp, 800);
    }
    


    if(Vision_Joint_Data_fetch_data.custom_controller_comm_recv == 0xff)
    {
        //这里加上UI反馈
        ui_cmd_send.custom_connect = 1;
    }
    else{
        ui_cmd_send.custom_connect = 0;
    }


    
}

/**
 * @brief 关节的键鼠控制
 * @attention shift反转 ctrl换小关节
 */
static void Joint_MouseKeySet()
{
    if(rc_data[TEMP].key[KEY_PRESS].z)
    {
        lift_cmd_send.left_angle = lift_fetch_data.now_left_angle + 200 * ramp_calc(&lift_l_ramp);
        lift_cmd_send.right_angle = lift_fetch_data.now_right_angle - 200 * ramp_calc(&lift_r_ramp);
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].z)
    {
        lift_cmd_send.left_angle = lift_fetch_data.now_left_angle - 200 * ramp_calc(&lift_l_ramp);
        lift_cmd_send.right_angle = lift_fetch_data.now_right_angle + 200 * ramp_calc(&lift_r_ramp);
    }
    else{
        lift_cmd_send.left_angle = lift_fetch_data.now_left_angle;
        lift_cmd_send.right_angle = lift_fetch_data.now_right_angle;
        ramp_init(&lift_l_ramp, 800);
        ramp_init(&lift_r_ramp, 800);
    }


    if(rc_data[TEMP].key[KEY_PRESS].x)
    {
        stretch_cmd_send.left_angle = stretch_fetch_data.now_left_angle + 100 * ramp_calc(&stretch_left_ramp);
        stretch_cmd_send.right_angle = stretch_fetch_data.now_right_angle - 100 * ramp_calc(&stretch_right_ramp);
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x)
    {
        stretch_cmd_send.left_angle = stretch_fetch_data.now_left_angle - 100 * ramp_calc(&stretch_left_ramp);
        stretch_cmd_send.right_angle = stretch_fetch_data.now_right_angle + 100 * ramp_calc(&stretch_right_ramp);
    }
    else{
        stretch_cmd_send.left_angle = stretch_fetch_data.now_left_angle;
        stretch_cmd_send.right_angle = stretch_fetch_data.now_right_angle;
        ramp_init(&stretch_left_ramp, 800);
        ramp_init(&stretch_right_ramp, 800);
    }


    //大ROLL左水平
    if (rc_data[TEMP].key[KEY_PRESS].b){
        BIG_ROLL_cmd_send.angle = BIG_ROLL_fetch_data.now_angle + 0.3 * ramp_calc(&BIG_ROLL_ramp);
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].b)
    {
        BIG_ROLL_cmd_send.angle = BIG_ROLL_fetch_data.now_angle - 0.3 * ramp_calc(&BIG_ROLL_ramp);
    }
    else{
        BIG_ROLL_cmd_send.angle = BIG_ROLL_fetch_data.now_angle;
        ramp_init(&BIG_ROLL_ramp, 400);
    }


    //YAW
    if (rc_data[TEMP].key[KEY_PRESS].g){
        YAW_cmd_send.angle = YAW_fetch_data.now_angle - 0.12 * ramp_calc(&YAW_ramp);
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].g)
    {
        YAW_cmd_send.angle = YAW_fetch_data.now_angle + 0.12 * ramp_calc(&YAW_ramp);
    }
    else{
        YAW_cmd_send.angle = YAW_fetch_data.now_angle;
        ramp_init(&YAW_ramp, 400);
    }


    //BIG_PITCH
    if (rc_data[TEMP].key[KEY_PRESS].f){
        BigPitch_cmd_send.angle  = BigPitch_fetch_data.now_angle - 0.3 * ramp_calc(&BIG_PITCH_ramp); 
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].f)
    {
        BigPitch_cmd_send.angle  = BigPitch_fetch_data.now_angle + 0.3 * ramp_calc(&BIG_PITCH_ramp); 
    }
    else{
        BigPitch_cmd_send.angle  = BigPitch_fetch_data.now_angle;
        ramp_init(&BIG_PITCH_ramp, 1000);
    }


    //SMALL_PITCH
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].f) {
        SMALL_PITCH_cmd_send.angle = SMALL_PITCH_fetch_data.now_angle - 0.2 * ramp_calc(&SMALL_PITCH_ramp);
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].f)
    {
        SMALL_PITCH_cmd_send.angle = SMALL_PITCH_fetch_data.now_angle + 0.2 * ramp_calc(&SMALL_PITCH_ramp);
    }
    else
    {
        SMALL_PITCH_cmd_send.angle = SMALL_PITCH_fetch_data.now_angle;
        ramp_init(&SMALL_PITCH_ramp, 1000);
    }


    //SMALL_ROLL
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].b) {

        SMALL_ROLL_cmd_send.speed = 8000;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].b) 
    {

        SMALL_ROLL_cmd_send.speed = -8000;
    }
    else
    {
        SMALL_ROLL_cmd_send.speed = 0;
    }
}


/**
 * @brief 控制器和键鼠的通用控制
 * 
 */
static void General_Control()
{
    //ctrl加w  s控制图传升降
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w)
    {
        GIMBAL_Data_cmd_send.angle = GIMBAL_fetch_data.now_angle - 400 * ramp_calc(&GIMBAL_ramp);
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s)
    {
        GIMBAL_Data_cmd_send.angle = GIMBAL_fetch_data.now_angle + 400 * ramp_calc(&GIMBAL_ramp);
    }
    else{
        ramp_init(&GIMBAL_ramp, 1200);
    }
    if(rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a)
    {
        servo_cmd_send.yaw_angle += 0.08;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d)
    {
        servo_cmd_send.yaw_angle -= 0.08;
    }
    if(rc_data[TEMP].mouse.x > 1)
    {
        servo_cmd_send.pitch_angle += 0.04;
    }
    else if (rc_data[TEMP].mouse.x < -1)
    {
        servo_cmd_send.pitch_angle -= 0.04;
    }
    // if(rc_data[TEMP].mouse.y > 2)
    // {
    //     servo_cmd_send.pitch_angle += 0.04;
    // }
    // else if (rc_data[TEMP].mouse.y < -2)
    // {
    //     servo_cmd_send.pitch_angle -= 0.04;
    // }
    
}

/**
 * @brief 一控多关节的函数，后续会移植到自动模式，它只控制末端姿态在xy轴平面的移动
 * @attention yaw到末端22cm
 */
float left_angle,right_angle;
void Multi_Joint_Motion_Solution()
{
    //暂时是遥控器决定启停
    if((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        BIG_ROLL_cmd_send.angle = 1.570795;
        SMALL_PITCH_cmd_send.angle = -1.470795;
        YAW_cmd_send.angle = - rc_data[TEMP].rc.rocker_r_ / 660.0 * 0.15 + YAW_fetch_data.now_angle;
        BigPitch_cmd_send.angle = 0;
       
        stretch_cmd_send.left_angle = left_angle + (1 - cos(YAW_fetch_data.now_angle)) * 18 / 6.4 * 360 * 11;
        stretch_cmd_send.right_angle = right_angle - (1 - cos(YAW_fetch_data.now_angle)) * 18 / 6.4 * 360 * 11;
    }
    else
    {
        left_angle = stretch_fetch_data.now_left_angle;
        right_angle = stretch_fetch_data.now_right_angle;
    }
}

/**
 * @brief 小斜坡
 * 
 * @param target 
 * @param measure 
 * @param expected_increments 
 * @param reference_size 
 * @return float 
 */
static float Automatic_mode_target_setting(float target, float measure, float expected_increments)
{
    float control_value;
    if (measure - target < -expected_increments) {
        control_value = measure + expected_increments;
    } else if (measure - target > expected_increments) {
        control_value = measure - expected_increments;
    } else {
        control_value = target;
    }
    return control_value;
}

/**
 * @brief 模式选择
 * 
 */
static void Automatic_mode_Decide()
{
    if(rc_data[TEMP].key[KEY_PRESS].z )     //中金
    {  
        auto_Assistant.type = 3;
        gold_auto[0].step1 = 1;
        gold_auto[0].step2 = 0;
        gold_auto[0].step3 = 0;
        gold_auto[0].step4 = 0;
        gold_auto[0].step5 = 0;
        gold_auto[0].step6 = 0;
        gold_auto[0].step7 = 0;
        gold_auto[0].step8 = 0;
        gold_auto[0].step9 = 0;
        gold_auto[0].step10 = 0;

        ui_cmd_send.auto_type = 1;
    }
    if(rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].z ) //左金
    {       
        auto_Assistant.type =2;
        gold_auto[1].step1 = 1;
        gold_auto[1].step2 = 0;
        gold_auto[1].step3 = 0;
        gold_auto[1].step4 = 0;
        gold_auto[1].step5 = 0;
        gold_auto[1].step6 = 0;
        gold_auto[1].step7 = 0;
        gold_auto[1].step8 = 0;
        gold_auto[1].step9 = 0;
        gold_auto[1].step10 = 0;

        ui_cmd_send.auto_type = 2;
    }
    if(rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].z)      //右金矿  
    {
        auto_Assistant.type =4;
        gold_auto[2].step1 = 1;
        gold_auto[2].step2 = 0;
        gold_auto[2].step3 = 0;
        gold_auto[2].step4 = 0;
        gold_auto[2].step5 = 0;
        gold_auto[2].step6 = 0;
        gold_auto[2].step7 = 0;
        gold_auto[2].step8 = 0;
        gold_auto[2].step9 = 0;
        gold_auto[2].step10 = 0;
        ui_cmd_send.auto_type = 3;

    }
    if(rc_data[TEMP].key[KEY_PRESS].r )    //自动抬金矿    
    {
        auto_Assistant.type = 6;
        pull_gold.step1 = 1;
        pull_gold.step2 = 0;
        pull_gold.step3 = 0;
        pull_gold.step4 = 0;
        pull_gold.step5 = 0;
        pull_gold.step6 = 0;
        pull_gold.step7 = 0;
        pull_gold.step8 = 0;
        pull_gold.step9 = 0;
        pull_gold.step10 = 0;
        ui_cmd_send.auto_type = 4;
    
    }
    //if(rc_data[TEMP].key[KEY_PRESS].x)        auto_Assistant.type =5;
    if(rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].r )//自动复位
    {
        auto_Assistant.type = 7;
        auto_reset.step1 = 1;
        auto_reset.step2 = 0;
        auto_reset.step3 = 0;
        auto_reset.step4 = 0;
        auto_reset.step5 = 0;
        auto_reset.step6 = 0;
        auto_reset.step7 = 0;
        auto_reset.step8 = 0;
        auto_reset.step9 = 0;
        auto_reset.step10 = 0;
        ui_cmd_send.auto_type = 5;        
    }


    //加一个自动复位标志用来解除限位
    if(auto_Assistant.type == 7)
    {
        auto_Assistant.auto_reset_flag = 2;
    }
    else{
        auto_Assistant.auto_reset_flag = 3;
    }



    if(rc_data[TEMP].key[KEY_PRESS].f)
    {
        auto_Assistant.type = 8;
        receive_mine.step1 = 1;
        receive_mine.step2 = 0;
        receive_mine.step3 = 0;
        receive_mine.step4 = 0;
        receive_mine.step5 = 0;
        receive_mine.step6 = 0;
        receive_mine.step7 = 0;
        receive_mine.step8 = 0;
        receive_mine.step9 = 0;
        receive_mine.step10 = 0;
        ui_cmd_send.auto_type = 6;
    }
    if(rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].f)
    {
        auto_Assistant.type = 9;
        pull_inter_mine.step1 = 1;
        pull_inter_mine.step2 = 0;
        pull_inter_mine.step3 = 0;
        pull_inter_mine.step4 = 0;
        pull_inter_mine.step5 = 0;
        pull_inter_mine.step6 = 0;
        pull_inter_mine.step7 = 0;
        pull_inter_mine.step8 = 0;
        pull_inter_mine.step9 = 0;
        pull_inter_mine.step10 = 0;
        ui_cmd_send.auto_type = 7;
    }
    if(rc_data[TEMP].key[KEY_PRESS].g)
    {
        auto_Assistant.type = 10;
        turtle[0].step1 = 1;
        turtle[0].step2 = 0;
        turtle[0].step3 = 0;
        turtle[0].step4 = 0;
        turtle[0].step5 = 0;
        turtle[0].step6 = 0;
        turtle[0].step7 = 0;
        turtle[0].step8 = 0;
        turtle[0].step9 = 0;
        turtle[0].step10 = 0;
        ui_cmd_send.auto_type = 8;
    }
    if(rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].g)
    {
        auto_Assistant.type = 11;
        turtle[1].step1 = 1;
        turtle[1].step2 = 0;
        turtle[1].step3 = 0;
        turtle[1].step4 = 0;
        turtle[1].step5 = 0;
        turtle[1].step6 = 0;
        turtle[1].step7 = 0;
        turtle[1].step8 = 0;
        turtle[1].step9 = 0;
        turtle[1].step10 = 0;
        ui_cmd_send.auto_type = 9;
    }
    if(rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].g)
    {
        auto_Assistant.type = 12;
        turtle[2].step1 = 1;
        turtle[2].step2 = 0;
        turtle[2].step3 = 0;
        turtle[2].step4 = 0;
        turtle[2].step5 = 0;
        turtle[2].step6 = 0;
        turtle[2].step7 = 0;
        turtle[2].step8 = 0;
        turtle[2].step9 = 0;
        turtle[2].step10 = 0;
        ui_cmd_send.auto_type = 10;
    }
    
}

/**
 * @brief 一键左金矿
 * 
 */
static void Auto_gold()
{
    switch (auto_Assistant.type)
    {
    case 3://中金
         if(gold_auto[0].step1 == 1)
    {
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.stretch_left,stretch_fetch_data.now_left_angle,200);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.stretch_right,stretch_fetch_data.now_right_angle,200);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.lift_left,lift_fetch_data.now_left_angle,150);
        lift_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.lift_right,lift_fetch_data.now_right_angle,150);
        if(lift_fetch_data.now_left_angle < initial_angle.lift_left + 200 && lift_fetch_data.now_left_angle > initial_angle.lift_left - 200 && stretch_fetch_data.now_left_angle > initial_angle.stretch_left - 350 && stretch_fetch_data.now_left_angle < initial_angle.stretch_left + 350)
        {
            gold_auto[0].step2 = 1;
            gold_auto[0].now_step = 2;
        }
        else{
            gold_auto[0].step2 = 0;
            gold_auto[0].now_step = 1;
        }
    }
    if(gold_auto[0].step2 == 1)
    {               

        BIG_ROLL_cmd_send.angle = Automatic_mode_target_setting(-1.49938965,BIG_ROLL_fetch_data.now_angle,0.4);
        SMALL_PITCH_cmd_send.angle = Automatic_mode_target_setting(0.0341424942,SMALL_PITCH_fetch_data.now_angle,0.3);
        YAW_cmd_send.angle = Automatic_mode_target_setting(0.08182621,YAW_fetch_data.now_angle,0.2);
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(-0.04348754880,BigPitch_fetch_data.now_angle,0.6);
        gold_auto[0].step1 = 0;
        if(YAW_fetch_data.now_angle > 0.08182621 - 0.005 && YAW_fetch_data.now_angle < 0.08182621 + 0.005 && BIG_ROLL_fetch_data.now_angle > -1.49938965 - 0.008 && BIG_ROLL_fetch_data.now_angle < -1.49938965 + 0.008 && BigPitch_fetch_data.now_angle > -0.04348754880 - 0.008 && BigPitch_fetch_data.now_angle < -0.04348754880 + 0.008 && SMALL_PITCH_fetch_data.now_angle < 0.0341424942 + 0.008 && SMALL_PITCH_fetch_data.now_angle > 0.0341424942 - 0.008)
        {
            gold_auto[0].step3 = 1;
            gold_auto[0].now_step = 3;
        }
        else{
            gold_auto[0].step3 = 0;
            gold_auto[0].now_step =2;
        }
    }
    if(gold_auto[0].step3 == 1)
    {
        gold_auto[0].step2 = 0;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(-6162.67334,stretch_fetch_data.now_left_angle,200);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(6232.36719,stretch_fetch_data.now_right_angle,200);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(-14344.8691,lift_fetch_data.now_left_angle,150);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(13964.3887,lift_fetch_data.now_right_angle,150);
        chassis_speed_scale.speed_change_flag = 3;
        ui_cmd_send.main_air_flag = 1;
        if (lift_fetch_data.now_left_angle < -14344.8691 + 200 && lift_fetch_data.now_left_angle > -14344.8691 - 200 && stretch_fetch_data.now_left_angle > 6232.36719 - 350 && stretch_fetch_data.now_left_angle < 6232.36719 + 350)
        {
            gold_auto[0].step4 = 1;
            gold_auto[0].now_step = 4;
        }
        else{
            gold_auto[0].step4 = 0;
            gold_auto[0].now_step = 3;
        }
    }
    if(gold_auto[0].step4 == 1)
    {
        gold_auto[0].step3 = 0;
    }
        break;
    case 2://左金
    if(gold_auto[1].step1 == 1)
    {
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.stretch_left,stretch_fetch_data.now_left_angle,200);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.stretch_right,stretch_fetch_data.now_right_angle,200);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.lift_left,lift_fetch_data.now_left_angle,150);
        lift_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.lift_right,lift_fetch_data.now_right_angle,150);
        if(lift_fetch_data.now_left_angle < initial_angle.lift_left + 200 && lift_fetch_data.now_left_angle > initial_angle.lift_left - 200 && stretch_fetch_data.now_left_angle > initial_angle.stretch_left - 350 && stretch_fetch_data.now_left_angle < initial_angle.stretch_left + 350)
        {
            gold_auto[1].step2 = 1;
            gold_auto[1].now_step = 2;
        }
        else{
            gold_auto[1].step2 = 0;
            gold_auto[1].now_step = 1;
        }
    }
    if(gold_auto[1].step2 == 1)
    {               

        BIG_ROLL_cmd_send.angle = Automatic_mode_target_setting(0.0696191788,BIG_ROLL_fetch_data.now_angle,0.4);
        SMALL_PITCH_cmd_send.angle = Automatic_mode_target_setting(-0.632677078,SMALL_PITCH_fetch_data.now_angle,0.3);
        YAW_cmd_send.angle = Automatic_mode_target_setting(0.96227169,YAW_fetch_data.now_angle,0.2);
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(-0.0679016113,BigPitch_fetch_data.now_angle,0.6);
        gold_auto[1].step1 = 0;
        if(YAW_fetch_data.now_angle > 0.96227169 - 0.005 && YAW_fetch_data.now_angle < 0.96227169 + 0.005 && BIG_ROLL_fetch_data.now_angle > 0.0696191788 - 0.008 && BIG_ROLL_fetch_data.now_angle < 0.0696191788 + 0.008 && BigPitch_fetch_data.now_angle > -0.0679016113 - 0.008 && BigPitch_fetch_data.now_angle < -0.0679016113 + 0.008 && SMALL_PITCH_fetch_data.now_angle < -0.632677078 + 0.008 && SMALL_PITCH_fetch_data.now_angle > -0.632677078 - 0.008)
        {
            gold_auto[1].step3 = 1;
            gold_auto[1].now_step = 3;
        }
        else{
            gold_auto[1].step3 = 0;
            gold_auto[1].now_step =2;
        }
    }
    if(gold_auto[1].step3 == 1)
    {
        gold_auto[1].step2 = 0;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(-7529.23828,stretch_fetch_data.now_left_angle,200);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(7444.86133,stretch_fetch_data.now_right_angle,200);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(-14344.8691,lift_fetch_data.now_left_angle,150);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(13964.3887,lift_fetch_data.now_right_angle,150);
        chassis_speed_scale.speed_change_flag = 3;
        ui_cmd_send.main_air_flag = 1;
        if (lift_fetch_data.now_left_angle < -14344.8691 + 200 && lift_fetch_data.now_left_angle > 14344.8691 - 200 && stretch_fetch_data.now_left_angle > -7529.23828 - 350 && stretch_fetch_data.now_left_angle < -7529.23828 + 350)
        {
            gold_auto[1].step4 = 1;
            gold_auto[1].now_step = 4;
        }
        else{
            gold_auto[1].step4 = 0;
            gold_auto[1].now_step = 3;
        }
    }
    if(gold_auto[1].step4 == 1)
    {
        gold_auto[1].step3 = 0;
    }
        break;


    case 4://右金
    
        if(gold_auto[2].step1 == 1)
    {
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.stretch_left,stretch_fetch_data.now_left_angle,200);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.stretch_right,stretch_fetch_data.now_right_angle,200);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.lift_left,lift_fetch_data.now_left_angle,150);
        lift_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.lift_right,lift_fetch_data.now_right_angle,150);
        if(lift_fetch_data.now_left_angle < initial_angle.lift_left + 200 && lift_fetch_data.now_left_angle > initial_angle.lift_left - 200 && stretch_fetch_data.now_left_angle > initial_angle.stretch_left - 350 && stretch_fetch_data.now_left_angle < initial_angle.stretch_left + 350)
        {
            gold_auto[2].step2 = 1;
            gold_auto[2].now_step = 2;
        }
        else{
            gold_auto[2].step2 = 0;
            gold_auto[2].now_step = 1;
        }
    }
    if(gold_auto[2].step2 == 1)
    {               

        BIG_ROLL_cmd_send.angle = Automatic_mode_target_setting(-0.131799698,BIG_ROLL_fetch_data.now_angle,0.4);
        SMALL_PITCH_cmd_send.angle = Automatic_mode_target_setting(1.03093719,SMALL_PITCH_fetch_data.now_angle,0.3);
        YAW_cmd_send.angle = Automatic_mode_target_setting(-0.788701057,YAW_fetch_data.now_angle,0.2);
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(-0.0907897949,BigPitch_fetch_data.now_angle,0.6);
        gold_auto[2].step1 = 0;
        if(YAW_fetch_data.now_angle > -0.788701057 - 0.005 && YAW_fetch_data.now_angle < -0.788701057 + 0.005 && BIG_ROLL_fetch_data.now_angle > -0.131799698 - 0.008 && BIG_ROLL_fetch_data.now_angle < -0.131799698 + 0.008 && BigPitch_fetch_data.now_angle > -0.0907897949 - 0.008 && BigPitch_fetch_data.now_angle < -0.0907897949 + 0.008 && SMALL_PITCH_fetch_data.now_angle > 1.03093719 - 0.008 && SMALL_PITCH_fetch_data.now_angle < 1.03093719 + 0.008)
        {
            gold_auto[2].step3 = 1;
            gold_auto[2].now_step = 3;
        }
        else{
            gold_auto[2].step3 = 0;
            gold_auto[2].now_step =2;
        }
    }
    if(gold_auto[2].step3 == 1)
    {
        gold_auto[2].step2 = 0;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(-7708.84424,stretch_fetch_data.now_left_angle,200);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(7743.68994,stretch_fetch_data.now_right_angle,200);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(-14344.8691,lift_fetch_data.now_left_angle,150);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(13964.3887,lift_fetch_data.now_right_angle,150);
        chassis_speed_scale.speed_change_flag = 3;
        ui_cmd_send.main_air_flag = 1;
        if (lift_fetch_data.now_left_angle < -14344.8691 + 200 && lift_fetch_data.now_left_angle > 14344.8691 - 200 && stretch_fetch_data.now_left_angle > -7708.84424 - 350 && stretch_fetch_data.now_left_angle < -7708.84424 + 350)
        {
            gold_auto[2].step4 = 1;
            gold_auto[2].now_step = 4;
        }
        else{
            gold_auto[2].step4 = 0;
            gold_auto[2].now_step = 3;
        }
    }
    if(gold_auto[2].step4 == 1)
    {
        gold_auto[2].step3 = 0;
    }
    

    default:
        break;
    }
    
}

/**
 * @brief 自动抬金矿
 * 
 */
static void Auto_Pull_gold()
{
    if(pull_gold.step1 == 1)
    {
        ui_cmd_send.main_air_flag = 1;
        chassis_speed_scale.speed_change_flag = 3;
        stretch_cmd_send.left_angle = stretch_fetch_data.now_left_angle + 20;
        stretch_cmd_send.right_angle  = stretch_fetch_data.now_right_angle - 20;
        if((stretch_fetch_data.now_left_current > 1100 || stretch_fetch_data.now_left_current < -1100) && stretch_fetch_data.now_left_speed < 6 && stretch_fetch_data.now_left_speed > -6)
        {
            pull_gold.step2 = 1;
            pull_gold.now_step = 2;
        }
        else{
            pull_gold.step2 = 0;
            pull_gold.now_step = 1;
        }
    }
    if(pull_gold.step2 == 1)
    {               

        lift_cmd_send.left_angle = Automatic_mode_target_setting(-11639.3018,lift_fetch_data.now_left_angle,80);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(12180.0479,lift_fetch_data.now_right_angle,80);
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-18157.8535,GIMBAL_fetch_data.now_angle, 500);
        pull_gold.step1 = 0;
        if(lift_fetch_data.now_left_angle < -11139.3018 + 150 && lift_fetch_data.now_left_angle > -11139.3018 -150)
        {
            pull_gold.step3 = 1;
            pull_gold.now_step = 3;
        }
        else{
            pull_gold.step3 = 0;
            pull_gold.now_step =2;
        }
    }
    if(pull_gold.step3 == 1)
    {
        pull_gold.step2 = 0;
    }
}

/**
 * @brief 自动银矿
 * 
 */
static void Auto_Sliver()
{

}

/**
 * @brief 自动复位
 * 
 */
 static void Auto_Reset()
 {
    if(auto_reset.step1 == 1)
    {
        stretch_cmd_send.left_angle = stretch_fetch_data.now_left_angle + 120;
        stretch_cmd_send.right_angle  = stretch_fetch_data.now_right_angle - 120;
        lift_cmd_send.left_angle = lift_fetch_data.now_left_angle + 320;
        lift_cmd_send.right_angle = lift_fetch_data.now_right_angle - 320;

        GIMBAL_Data_cmd_send.angle = GIMBAL_fetch_data.now_angle + 500;

        if((stretch_fetch_data.now_left_current > 2200 || stretch_fetch_data.now_left_current < -2200) && stretch_fetch_data.now_left_speed < 3 && stretch_fetch_data.now_left_speed > -3 && (lift_fetch_data.now_left_current > 2200 || lift_fetch_data.now_left_current < -2200) && lift_fetch_data.now_left_speed < 3 && lift_fetch_data.now_left_speed > -3 && ((GIMBAL_fetch_data.current > 3700 || GIMBAL_fetch_data.current < - 3700) && (GIMBAL_fetch_data.speed < 2 || GIMBAL_fetch_data.speed > -2)))
        {
            auto_reset.step2 = 1;
            auto_reset.now_step = 2;
        }
        else{
            auto_reset.step2 = 0;
            auto_reset.now_step = 1;
        }
    }
    if(auto_reset.step2 == 1)
    {
        auto_reset.step1 = 0;
         
        __set_FAULTMASK(1);
        NVIC_SystemReset();
        auto_Assistant.auto_reset_flag = 3;
        auto_reset.step2 = 0;
    }
 }
int16_t delay_flag = 0;
/**
 * @brief 自动放矿
 * 
 */
static void Auto_Receive()
{
    if(receive_mine.step1 == 1)
    {
        ui_cmd_send.main_air_flag = 1;
        ui_cmd_send.mine_air_flag = 1;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.stretch_left,stretch_fetch_data.now_left_angle,300);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.stretch_right,stretch_fetch_data.now_right_angle,300);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.lift_left,lift_fetch_data.now_left_angle,250);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(initial_angle.lift_right,lift_fetch_data.now_right_angle,250);
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-8262.33398,GIMBAL_fetch_data.now_angle, 500);
        if(lift_fetch_data.now_left_angle < initial_angle.lift_left + 200 && lift_fetch_data.now_left_angle > initial_angle.lift_left - 200 && stretch_fetch_data.now_left_angle > initial_angle.stretch_left - 350 && stretch_fetch_data.now_left_angle < initial_angle.stretch_left + 350)
        {
            receive_mine.step2 = 1;
            receive_mine.now_step = 2;
        }
        else{
            receive_mine.step2 = 0;
            receive_mine.now_step = 1;
        }
    }
     if (receive_mine.step2 == 1)
    {
        receive_mine.step1 = 0;
        BIG_ROLL_cmd_send.angle = Automatic_mode_target_setting(-1.63633919,BIG_ROLL_fetch_data.now_angle,0.4);
        SMALL_PITCH_cmd_send.angle = Automatic_mode_target_setting(1.91825008,SMALL_PITCH_fetch_data.now_angle,0.5);
        YAW_cmd_send.angle = Automatic_mode_target_setting(0.0642786026,YAW_fetch_data.now_angle,0.4);
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(0,BigPitch_fetch_data.now_angle,0.6);
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-8262.33398,GIMBAL_fetch_data.now_angle, 500);
        if(YAW_fetch_data.now_angle > 0.0642786026 - 0.005 && YAW_fetch_data.now_angle < 0.0642786026 + 0.005 && BIG_ROLL_fetch_data.now_angle > -1.63633919 - 0.008 && BIG_ROLL_fetch_data.now_angle < -1.63633919 + 0.008 && BigPitch_fetch_data.now_angle > - 0.01 && BigPitch_fetch_data.now_angle < 0.01 && SMALL_PITCH_fetch_data.now_angle < 1.91825008 + 0.008 && SMALL_PITCH_fetch_data.now_angle > 1.91825008 - 0.008)
        {
            receive_mine.step3 = 1;
            receive_mine.now_step = 3;
        }
        else{
            receive_mine.step3 = 0;
            receive_mine.now_step =2;
        }
    }
    if (receive_mine.step3 == 1)
    {
        receive_mine.step2 = 0;
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(1.5220871,BigPitch_fetch_data.now_angle,0.3);
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-8262.33398,GIMBAL_fetch_data.now_angle, 500);
        if(BigPitch_fetch_data.now_angle > 1.5220871 - 0.01 && BigPitch_fetch_data.now_angle < 1.5220871 + 0.01)
        {
            receive_mine.step4 = 1;
            receive_mine.now_step = 4;
        }
        else{
            receive_mine.step4 = 0;
            receive_mine.now_step =3;
        }
    }
    if(receive_mine.step4 == 1)
    {
        receive_mine.step3 = 0;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(-927.862427,stretch_fetch_data.now_left_angle,300);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(875.081909,stretch_fetch_data.now_right_angle,300);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(-3842.17468 ,lift_fetch_data.now_left_angle,350);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(3870.0332 ,lift_fetch_data.now_right_angle,350);
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-8262.33398,GIMBAL_fetch_data.now_angle, 500);
        if(((lift_fetch_data.now_left_angle < -3842.17468 + 100 && lift_fetch_data.now_left_angle > -3842.17468 - 100)||((lift_fetch_data.now_left_current > 2200 || lift_fetch_data.now_left_current < -2200) && lift_fetch_data.now_left_speed < 3 && lift_fetch_data.now_left_speed > -3)) && stretch_fetch_data.now_left_angle > -927.862427 - 150 && stretch_fetch_data.now_left_angle < -927.862427 + 150)
        {
            receive_mine.step5 = 1;
            receive_mine.now_step = 5;
        }
        else{
            receive_mine.step5 = 0;
            receive_mine.now_step = 4;
        }
    }
    if(receive_mine.step5 == 1)
    {
        receive_mine.step4 = 0;
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-8262.33398,GIMBAL_fetch_data.now_angle, 500);
        delay_flag++;
        if(delay_flag > 500)
        {
            ui_cmd_send.main_air_flag = 2;
        }
        if(delay_flag > 1000)
        {
            receive_mine.step6 = 1;
            receive_mine.now_step = 6;
            delay_flag = 0;
        }
        else
        {
            receive_mine.step6 = 0;
            receive_mine.now_step = 5;
        }

    }
    if(receive_mine.step6 == 1)
    {
        receive_mine.step5 = 0;
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-27573.6641,GIMBAL_fetch_data.now_angle, 500);
        //ui_cmd_send.main_air_flag = 2;
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(0.476844788,BigPitch_fetch_data.now_angle,0.4);        
        if(BigPitch_fetch_data.now_angle < 0.476844788 + 0.008 && BigPitch_fetch_data.now_angle > 0.476844788 - 0.008)
        {
            receive_mine.step7 = 1;
            receive_mine.now_step = 7;
        }
        else
        {
            receive_mine.step7 = 0;
            receive_mine.now_step = 6;
        }

    }
    if(receive_mine.step7 == 1)
    {
        receive_mine.step6 = 0;
        BIG_ROLL_cmd_send.angle = Automatic_mode_target_setting(-1.63633919,BIG_ROLL_fetch_data.now_angle,0.4);
        SMALL_PITCH_cmd_send.angle = Automatic_mode_target_setting(1.12516212,SMALL_PITCH_fetch_data.now_angle,0.3);
        YAW_cmd_send.angle = Automatic_mode_target_setting(-0.432677078,YAW_fetch_data.now_angle,0.4);
       
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-27573.6641,GIMBAL_fetch_data.now_angle, 500);
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(-6962.34521,stretch_fetch_data.now_left_angle,300);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(7015.25244,stretch_fetch_data.now_right_angle,300);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(-7225.22705 ,lift_fetch_data.now_left_angle,350);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(7457.42969 ,lift_fetch_data.now_right_angle,350);
        chassis_speed_scale.speed_change_flag = 4;

    }
}


/**
 * @brief 自动从肚子拿出
 * 
 */

uint16_t delay_flag1 = 0;
static void Auto_Pull_Out()
{
    if(pull_inter_mine.step1 == 1)
    {
        ui_cmd_send.main_air_flag = 1;
        ui_cmd_send.mine_air_flag = 1;
        BIG_ROLL_cmd_send.angle = Automatic_mode_target_setting(-1.63633919,BIG_ROLL_fetch_data.now_angle,0.4);
        SMALL_PITCH_cmd_send.angle = Automatic_mode_target_setting(1.57072544,SMALL_PITCH_fetch_data.now_angle,0.5);
        YAW_cmd_send.angle = Automatic_mode_target_setting(0.0642786026,YAW_fetch_data.now_angle,0.4);
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-8262.33398,GIMBAL_fetch_data.now_angle, 500);
        //BigPitch_cmd_send.angle = Automatic_mode_target_setting(0,BigPitch_fetch_data.now_angle,0.3);
        
        if(YAW_fetch_data.now_angle > 0.0642786026 - 0.005 && YAW_fetch_data.now_angle < 0.0642786026 + 0.005 && BIG_ROLL_fetch_data.now_angle > -1.63633919 - 0.008 && BIG_ROLL_fetch_data.now_angle < -1.63633919 + 0.008 && SMALL_PITCH_fetch_data.now_angle < 1.57072544 + 0.008 && SMALL_PITCH_fetch_data.now_angle > 1.57072544 - 0.008)
        {
            pull_inter_mine.step2 = 1;
            pull_inter_mine.now_step = 2;
        }
        else{
            pull_inter_mine.step2 = 0;
            pull_inter_mine.now_step =1;
        }
    }  
    if (pull_inter_mine.step2 == 1)
    {
        pull_inter_mine.step1 = 0;
        ui_cmd_send.main_air_flag = 1;
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-8262.33398,GIMBAL_fetch_data.now_angle, 500);
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(0,BigPitch_fetch_data.now_angle,0.4);
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.stretch_left,stretch_fetch_data.now_left_angle,300);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.stretch_right,stretch_fetch_data.now_right_angle,300);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.lift_left,lift_fetch_data.now_left_angle,250);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(initial_angle.lift_right,lift_fetch_data.now_right_angle,250);
        if(lift_fetch_data.now_left_angle < initial_angle.lift_left + 200 && lift_fetch_data.now_left_angle > initial_angle.lift_left - 200 && stretch_fetch_data.now_left_angle > initial_angle.stretch_left - 350 && stretch_fetch_data.now_left_angle < initial_angle.stretch_left + 350 && BigPitch_fetch_data.now_angle < 0.008 && BigPitch_fetch_data.now_angle > - 0.008)
        {
            pull_inter_mine.step3 = 1;
            pull_inter_mine.now_step = 3;
        }
        else{
            pull_inter_mine.step3 = 0;
            pull_inter_mine.now_step = 2;
        }
    }
    if(pull_inter_mine.step3 == 1)
    {
        pull_inter_mine.step2 = 0;
        ui_cmd_send.main_air_flag = 1;
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-8262.33398,GIMBAL_fetch_data.now_angle, 500);
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(1.47478485,BigPitch_fetch_data.now_angle,0.4);
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(-1927.862427,stretch_fetch_data.now_left_angle,300);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(1875.081909,stretch_fetch_data.now_right_angle,300);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(-2342.17468 ,lift_fetch_data.now_left_angle,350);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(2370.0332 ,lift_fetch_data.now_right_angle,350);
        if(BigPitch_fetch_data.now_angle > 1.47478485 - 0.01 && BigPitch_fetch_data.now_angle < 1.47478485 + 0.01 && lift_fetch_data.now_left_angle < -2342.17468 + 100 && lift_fetch_data.now_left_angle > -2342.17468 - 100 && ((stretch_fetch_data.now_left_angle > -1927.862427 - 150 && stretch_fetch_data.now_left_angle < -1927.862427 + 150) || ((stretch_fetch_data.now_left_current > 2000 || stretch_fetch_data.now_left_current < -2000) && stretch_fetch_data.now_left_speed < 3 && stretch_fetch_data.now_left_speed > -3)))

        {
            pull_inter_mine.step4 = 1;
            pull_inter_mine.now_step = 4;
        }
        else{
            pull_inter_mine.step4 = 0;
            pull_inter_mine.now_step =3;
        }

    }
    if(pull_inter_mine.step4 == 1)
    {
        pull_inter_mine.step3 = 0;
        ui_cmd_send.main_air_flag = 1;
        ui_cmd_send.mine_air_flag = 2;
        delay_flag1++;
        if(delay_flag1 > 800)
        {
            pull_inter_mine.step5 = 1;
            pull_inter_mine.now_step = 5;
            delay_flag1 = 0;
        }
        else{
            pull_inter_mine.step5 = 0;
            pull_inter_mine.now_step = 4;
        }

    }
    if(pull_inter_mine.step5 == 1)
    {
        pull_inter_mine.step4 = 0;
        SMALL_PITCH_cmd_send.angle = Automatic_mode_target_setting(1.91825008,SMALL_PITCH_fetch_data.now_angle,0.08);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.lift_left,lift_fetch_data.now_left_angle,300);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(initial_angle.lift_right,lift_fetch_data.now_right_angle,300);
        if(SMALL_PITCH_fetch_data.now_angle < 1.91825008 + 0.008 && SMALL_PITCH_fetch_data.now_angle > 1.91825008 - 0.008 && lift_fetch_data.now_left_angle < initial_angle.lift_left + 200 && lift_fetch_data.now_left_angle > initial_angle.lift_left - 200)
        {
            pull_inter_mine.step6 = 1;
            pull_inter_mine.now_step = 6;
        }
        else{
            pull_inter_mine.step6 = 0;
            pull_inter_mine.now_step = 5;
        }

       
    }
    if(pull_inter_mine.step6 == 1)
    {
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-26592.1445,GIMBAL_fetch_data.now_angle, 500);
        pull_inter_mine.step5 = 0;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.stretch_left,stretch_fetch_data.now_left_angle,300);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.stretch_right,stretch_fetch_data.now_right_angle,300);
        //lift_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.lift_left,lift_fetch_data.now_left_angle,300);
        //lift_cmd_send.right_angle = Automatic_mode_target_setting(initial_angle.lift_right,lift_fetch_data.now_right_angle,300);
        chassis_speed_scale.speed_change_flag = 3;

    //     if(lift_fetch_data.now_left_angle < initial_angle.lift_left + 200 && lift_fetch_data.now_left_angle > initial_angle.lift_left - 200 && stretch_fetch_data.now_left_angle > initial_angle.stretch_left - 150 && stretch_fetch_data.now_left_angle < initial_angle.stretch_left + 150)
    //     {
    //         receive_mine.step6 = 1;
    //         receive_mine.now_step = 6;
    //     }
    //     else{
    //         receive_mine.step6 = 0;
    //         receive_mine.now_step = 5;
    //     }
    // }
    // if(pull_inter_mine.step6 == 1)
    // {
    //     pull_inter_mine.step5 = 0;
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(0.4,BigPitch_fetch_data.now_angle,0.3);
    }
              
}

/**
 * @brief 无矿乌龟
 * 
 */
static void Auto_NO_Mine_Turtle()
{
    GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-18600.1621,GIMBAL_fetch_data.now_angle, 500);
    if(turtle[0].step1 == 1)
    {
        ui_cmd_send.main_air_flag = 2;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.stretch_left,stretch_fetch_data.now_left_angle,400);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.stretch_right,stretch_fetch_data.now_right_angle,400);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.lift_left,lift_fetch_data.now_left_angle,350);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(initial_angle.lift_right,lift_fetch_data.now_right_angle,350);
        
        if(lift_fetch_data.now_left_angle < initial_angle.lift_left + 200 && lift_fetch_data.now_left_angle > initial_angle.lift_left - 200 && stretch_fetch_data.now_left_angle > initial_angle.stretch_left - 350 && stretch_fetch_data.now_left_angle < initial_angle.stretch_left + 350)
        {
            turtle[0].step2 = 1;
            turtle[0].now_step = 2;
        }
        else{
            turtle[0].step2 = 0;
            turtle[0].now_step = 1;
        }
    }
    if(turtle[0].step2 == 1)
    {
        turtle[0].step1 = 0;
        ui_cmd_send.main_air_flag = 2;
        BIG_ROLL_cmd_send.angle = Automatic_mode_target_setting(-1.63633919,BIG_ROLL_fetch_data.now_angle,0.4);
        SMALL_PITCH_cmd_send.angle = Automatic_mode_target_setting(1.28995991,SMALL_PITCH_fetch_data.now_angle,0.5);
        YAW_cmd_send.angle = Automatic_mode_target_setting(1.03818607,YAW_fetch_data.now_angle,0.4);
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(0.348667145,BigPitch_fetch_data.now_angle,0.4);
        if(YAW_fetch_data.now_angle > 1.03818607 - 0.005 && YAW_fetch_data.now_angle < 1.03818607 + 0.005 && BIG_ROLL_fetch_data.now_angle > -1.63633919 - 0.008 && BIG_ROLL_fetch_data.now_angle < -1.63633919 + 0.008 && BigPitch_fetch_data.now_angle > 0.348667145 - 0.008 && BigPitch_fetch_data.now_angle < 0.348667145 + 0.008 && SMALL_PITCH_fetch_data.now_angle < 1.28995991 + 0.008 && SMALL_PITCH_fetch_data.now_angle > 1.28995991 - 0.008)
        {
            turtle[0].step3 = 1;
            turtle[0].now_step = 3;
        }
        else{
            turtle[0].step3 = 0;
            turtle[0].now_step =2;
        }
    }
    if(turtle[0].step3 == 1)
    {
        turtle[0].step2 = 0;
        //ui_cmd_send.main_air_flag = 2;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(-7314.34766,stretch_fetch_data.now_left_angle,400);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(7483.92871,stretch_fetch_data.now_right_angle,400);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(-13688.3086,lift_fetch_data.now_left_angle,350);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(14100.6006,lift_fetch_data.now_right_angle,350);
        chassis_speed_scale.speed_change_flag = 4;
    }

} 

/**
 * @brief 有矿乌龟
 * 
 */
static void Auto_ONE_Mine_BUT_Warehouse_NO_Mine_Turtle()
{
    GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-18600.1621,GIMBAL_fetch_data.now_angle, 500);
    if(turtle[1].step1 == 1)
    {
        ui_cmd_send.main_air_flag = 1;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.stretch_left,stretch_fetch_data.now_left_angle,400);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.stretch_right,stretch_fetch_data.now_right_angle,400);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.lift_left,lift_fetch_data.now_left_angle,350);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(initial_angle.lift_right,lift_fetch_data.now_right_angle,350);
        
        if(lift_fetch_data.now_left_angle < initial_angle.lift_left + 200 && lift_fetch_data.now_left_angle > initial_angle.lift_left - 200 && stretch_fetch_data.now_left_angle > initial_angle.stretch_left - 350 && stretch_fetch_data.now_left_angle < initial_angle.stretch_left + 350)
        {
            turtle[1].step2 = 1;
            turtle[1].now_step = 2;
        }
        else{
            turtle[1].step2 = 0;
            turtle[1].now_step = 1;
        }
    }
    if(turtle[1].step2 == 1)
    {
        ui_cmd_send.main_air_flag = 1;
        turtle[1].step1 = 0;
        BIG_ROLL_cmd_send.angle = Automatic_mode_target_setting(-0.0093460083,BIG_ROLL_fetch_data.now_angle,0.4);
        SMALL_PITCH_cmd_send.angle = Automatic_mode_target_setting(-1.86217308,SMALL_PITCH_fetch_data.now_angle,0.5);
        YAW_cmd_send.angle = Automatic_mode_target_setting(1.18314648,YAW_fetch_data.now_angle,0.4);
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(-0.438697815,BigPitch_fetch_data.now_angle,0.4);
        if(YAW_fetch_data.now_angle > 1.18314648 - 0.005 && YAW_fetch_data.now_angle < 1.18314648 + 0.005 && BIG_ROLL_fetch_data.now_angle > -0.0093460083 - 0.008 && BIG_ROLL_fetch_data.now_angle < -0.0093460083 + 0.008 && BigPitch_fetch_data.now_angle > -0.438697815 - 0.008 && BigPitch_fetch_data.now_angle < -0.438697815 + 0.008 && SMALL_PITCH_fetch_data.now_angle < -1.86217308 + 0.008 && SMALL_PITCH_fetch_data.now_angle > -1.86217308 - 0.008)
        {
            turtle[1].step3 = 1;
            turtle[1].now_step = 3;
        }
        else{
            turtle[1].step3 = 0;
            turtle[1].now_step =2;
        }
    }
    if(turtle[1].step3 == 1)
    {
        //ui_cmd_send.main_air_flag = 1;
        turtle[1].step2 = 0;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(-7314.34766,stretch_fetch_data.now_left_angle,400);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(7483.92871,stretch_fetch_data.now_right_angle,400);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(-14137.6484,lift_fetch_data.now_left_angle,350);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(14122.3086,lift_fetch_data.now_right_angle,350);
        chassis_speed_scale.speed_change_flag = 4;
    }
}

static void Auto_Double_Mine()
{
     GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(-18600.1621,GIMBAL_fetch_data.now_angle, 500);
    if(turtle[2].step1 == 1)
    {
        ui_cmd_send.main_air_flag = 1;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.stretch_left,stretch_fetch_data.now_left_angle,400);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(initial_angle.stretch_right,stretch_fetch_data.now_right_angle,400);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(initial_angle.lift_left,lift_fetch_data.now_left_angle,350);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(initial_angle.lift_right,lift_fetch_data.now_right_angle,350);
        
        if(lift_fetch_data.now_left_angle < initial_angle.lift_left + 200 && lift_fetch_data.now_left_angle > initial_angle.lift_left - 200 && stretch_fetch_data.now_left_angle > initial_angle.stretch_left - 350 && stretch_fetch_data.now_left_angle < initial_angle.stretch_left + 350)
        {
            turtle[2].step2 = 1;
            turtle[2].now_step = 2;
        }
        else{
            turtle[2].step2 = 0;
            turtle[2].now_step = 1;
        }
    }
    if(turtle[2].step2 == 1)
    {
        turtle[2].step1 = 0;
        ui_cmd_send.main_air_flag = 1;
        BIG_ROLL_cmd_send.angle = Automatic_mode_target_setting(-1.8568325,BIG_ROLL_fetch_data.now_angle,0.4);
        SMALL_PITCH_cmd_send.angle = Automatic_mode_target_setting(-1.5611887,SMALL_PITCH_fetch_data.now_angle,0.5);
        YAW_cmd_send.angle = Automatic_mode_target_setting(0.903142929,YAW_fetch_data.now_angle,0.4);
        BigPitch_cmd_send.angle = Automatic_mode_target_setting(0.464637756,BigPitch_fetch_data.now_angle,0.4);
        if(YAW_fetch_data.now_angle > 0.903142929 - 0.005 && YAW_fetch_data.now_angle < 0.903142929 + 0.005 && BIG_ROLL_fetch_data.now_angle > -1.8568325 - 0.008 && BIG_ROLL_fetch_data.now_angle < -1.8568325 + 0.008 && BigPitch_fetch_data.now_angle > 0.464637756 - 0.008 && BigPitch_fetch_data.now_angle < 0.464637756 + 0.008 && SMALL_PITCH_fetch_data.now_angle < -1.5611887 + 0.008 && SMALL_PITCH_fetch_data.now_angle > -1.5611887 - 0.008)
        {
            turtle[2].step3 = 1;
            turtle[2].now_step = 3;
        }
        else{
            turtle[2].step3 = 0;
            turtle[2].now_step =2;
        }
    }
    if(turtle[2].step3 == 1)
    {
        turtle[2].step2 = 0;
        //ui_cmd_send.main_air_flag = 1;
        stretch_cmd_send.left_angle = Automatic_mode_target_setting(-6962.34521,stretch_fetch_data.now_left_angle,300);
        stretch_cmd_send.right_angle  = Automatic_mode_target_setting(7015.25244,stretch_fetch_data.now_right_angle,300);
        lift_cmd_send.left_angle = Automatic_mode_target_setting(-7225.22705 ,lift_fetch_data.now_left_angle,350);
        lift_cmd_send.right_angle = Automatic_mode_target_setting(7457.42969 ,lift_fetch_data.now_right_angle,350);
        chassis_speed_scale.speed_change_flag = 4;
    }
}

/**
 * @brief 统一的模式管理
 * 
 */
static void Auto_Mode()
{
    Automatic_mode_Decide();
    SMALL_ROLL_cmd_send.speed = Vision_Joint_Data_fetch_data.vision_small_roll;
    switch (auto_Assistant.type)
    {
    case 2:
    case 3:
    case 4:
        Auto_gold();
        GIMBAL_Data_cmd_send.angle = Automatic_mode_target_setting(initial_angle.gimbal,GIMBAL_fetch_data.now_angle, 500);
        break;
    case 5:
        Auto_Sliver();
        break;
    case 6:
        Auto_Pull_gold();
        break;
    case 7:
        Auto_Reset();
        break;
    case 8:
        Auto_Receive();
        break;
    case 9:
        Auto_Pull_Out();
        break;
    case 10:
        Auto_NO_Mine_Turtle();
        break;
    case 11:
        Auto_ONE_Mine_BUT_Warehouse_NO_Mine_Turtle();
        break;
    case 12:
        Auto_Double_Mine();
        break;

    default:
        break;
    }
  
}

static int8_t last_mode;
/**
 * @brief CMD核心任务
 *
 */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(lift_feed_sub, (void *)&lift_fetch_data);
    SubGetMessage(stretch_feed_sub, (void *)&stretch_fetch_data);
    SubGetMessage(servo_feed_sub, (void *)&servo_fetch_data);
    SubGetMessage(BigPitch_feed_sub, (void *)&BigPitch_fetch_data);
    SubGetMessage(YAW_feed_sub, (void *)&YAW_fetch_data);
    SubGetMessage(BIG_ROLL_feed_sub, (void *)&BIG_ROLL_fetch_data);
    SubGetMessage(SMALL_PITCH_feed_sub, (void *)&SMALL_PITCH_fetch_data);
    SubGetMessage(SMALL_ROLL_feed_sub, (void *)&SMALL_ROLL_fetch_data);
    SubGetMessage(vision_joint_data_sub, (void *)&Vision_Joint_Data_fetch_data);
    SubGetMessage(GIMBAL_feed_sub, (void *)&GIMBAL_fetch_data);


    Emergency_Handling();

    //加上温度回传
    ui_cmd_send.big_pitch_temputure = BigPitch_fetch_data.temp;

    //加入手动刷新UI
   
    last_mode = mode.control_mode;

    if(rc_data[TEMP].mouse.press_l)
    {
        mode.control_mode = 2;
    }
    else if (rc_data[TEMP].mouse.press_r)
    {
        mode.control_mode = 3;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].ctrl)
    {
        mode.control_mode = 4;
    }
   //防模式切换干扰
    
    if ((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        General_Control();
        Chassis_MouseKeySet();
        if(mode.control_mode == 2)
        {
            ui_cmd_send.Control_mode = 0;
            Vision_Control();
        }
        else if (mode.control_mode == 3)
        {
            ui_cmd_send.Control_mode = 1;
            Auto_Mode();
        }
        else if (mode.control_mode == 4)
        {
            ui_cmd_send.Control_mode = 2;
            Joint_MouseKeySet();
        }
    }
    else{
        RemoteControlSet();
    }
    if(last_mode != 3 && mode.control_mode == 3)
    {
        auto_Assistant.type = 21;
        BigPitch_cmd_send.angle = BigPitch_fetch_data.now_angle;
        YAW_cmd_send.angle = YAW_fetch_data.now_angle;
        SMALL_PITCH_cmd_send.angle = SMALL_PITCH_fetch_data.now_angle;
        BIG_ROLL_cmd_send.angle = BIG_ROLL_fetch_data.now_angle;
        lift_cmd_send.left_angle = lift_fetch_data.now_left_angle;
        lift_cmd_send.right_angle = lift_fetch_data.now_right_angle;
        stretch_cmd_send.left_angle = stretch_fetch_data.now_left_angle;
        stretch_cmd_send.right_angle = stretch_fetch_data.now_right_angle;
        ui_cmd_send.auto_type = 0;
    }

    //Multi_Joint_Motion_Solution();
    if (auto_Assistant.auto_reset_flag == 3)
    {
        Stall_Detection();
        
    }
    //里面内含复位解放
    cmd_value_limit();

    air_controll();


    PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(lift_cmd_pub, (void *)&lift_cmd_send);
    PubPushMessage(stretch_cmd_pub, (void *)&stretch_cmd_send);
    PubPushMessage(servo_cmd_pub, (void *)&servo_cmd_send);
    PubPushMessage(BigPitch_cmd_pub, (void *)&BigPitch_cmd_send);
    PubPushMessage(YAW_cmd_pub, (void *)&YAW_cmd_send);
    PubPushMessage(BIG_ROLL_cmd_pub, (void *)&BIG_ROLL_cmd_send);
    PubPushMessage(SMALL_PITCH_cmd_pub, (void *)&SMALL_PITCH_cmd_send);
    PubPushMessage(SMALL_ROLL_cmd_pub, (void *)&SMALL_ROLL_cmd_send);
    PubPushMessage(vision_joint_data_pub, (void *)&Vision_Joint_Data_cmd_send);
    PubPushMessage(GIMBAL_cmd_pub, (void *)&GIMBAL_Data_cmd_send);



}