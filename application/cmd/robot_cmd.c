// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "ramp.h"
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
static Subscriber_t *ui_feed_sub;
static ui_Cmd_s ui_cmd_send;
static ui_Upload_Data_s ui_fetch_data;

static Publisher_t *vision_joint_data_pub;
static Subscriber_t *vision_joint_data_sub;
static Vision_Joint_Data_Upload_Data_s Vision_Joint_Data_fetch_data; 
static Vision_Joint_Data_Ctrl_Cmd_s Vision_Joint_Data_cmd_send;  

extern int mouse_count_r;
struct 
{
    float stretch_left;
    float stretch_right;
    
}initial_angle;//便于统计最初的必要关节角度角度;

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
}


/**
 * @brief CMD初始化
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

    ui_cmd_pub  = PubRegister("ui_cmd", sizeof(ui_Cmd_s));
    ui_feed_sub = SubRegister("ui_feed", sizeof(ui_Upload_Data_s));

    vision_joint_data_pub  = PubRegister("vision_joint_data_cmd", sizeof(Vision_Joint_Data_Ctrl_Cmd_s));
    vision_joint_data_sub = SubRegister("vision_joint_data_feed", sizeof(Vision_Joint_Data_Upload_Data_s));

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

    lift_cmd_send.left_angle   = Limit_Set(lift_cmd_send.left_angle, -508.976105, -13802.6094);
    lift_cmd_send.right_angle  = Limit_Set(lift_cmd_send.right_angle, 13762.1768, 428.773926);

    stretch_cmd_send.left_angle   = Limit_Set(stretch_cmd_send.left_angle, -768.91333,-13713.7959);
    stretch_cmd_send.right_angle = Limit_Set(stretch_cmd_send.right_angle,13731.8115,729.00872 );
   
    BIG_ROLL_cmd_send.angle = Limit_Set(BIG_ROLL_cmd_send.angle,1.927786,-1.873998);

    BigPitch_cmd_send.angle = Limit_Set(BigPitch_cmd_send.angle,3.36995316,-0.502784729);

    YAW_cmd_send.angle = Limit_Set(YAW_cmd_send.angle,1.721027,-1.78664);

    SMALL_PITCH_cmd_send.angle = Limit_Set(SMALL_PITCH_cmd_send.angle,1.84309864,-2.03803349);
}

/**
 * @brief 堵转检测
 * @todo 后续加入升降和伸出的堵转检测，目前给的限制值都是按照额定给的，后续要具体测量
 * 
 */

void Stall_Detection()
{
    SMALL_PITCH_cmd_send.angle = Stall(SMALL_PITCH_fetch_data.torque,SMALL_PITCH_fetch_data.now_angle,SMALL_PITCH_cmd_send.angle,3.5,SMALL_PITCH_fetch_data.speed,2);
    BigPitch_cmd_send.angle = Stall(BigPitch_fetch_data.torque,BigPitch_fetch_data.now_angle,BigPitch_cmd_send.angle,8,BigPitch_fetch_data.speed,1.4);
    BIG_ROLL_cmd_send.angle = Stall(BIG_ROLL_fetch_data.torque,BIG_ROLL_fetch_data.now_angle,BIG_ROLL_cmd_send.angle,3.5,BIG_ROLL_fetch_data.speed,1.1);
    YAW_cmd_send.angle = Stall(YAW_fetch_data.torque,YAW_fetch_data.now_angle,YAW_cmd_send.angle,7,YAW_fetch_data.speed,1);


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
    // 左侧开关状态[中],右侧开关状态[下]
    if ((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_mid(rc_data[TEMP].rc.switch_left)) {

        servo_cmd_send.pitch_angle = rc_data[TEMP].rc.rocker_r1 / 660.0 * 180;
        servo_cmd_send.yaw_angle   = rc_data[TEMP].rc.rocker_l_ / 660.0 * 90;
        if (servo_cmd_send.pitch_angle > 180) {
            servo_cmd_send.pitch_angle = 180;
        } else if (servo_cmd_send.pitch_angle < -180) {
            servo_cmd_send.pitch_angle = -180;
        }
        if (servo_cmd_send.yaw_angle > 21) {
            servo_cmd_send.yaw_angle = 21;
        } else if (servo_cmd_send.yaw_angle < -179) {
            servo_cmd_send.yaw_angle = -179;
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
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{

    servo_cmd_send.pitch_angle += (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w - rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s) * 0.5;
    servo_cmd_send.yaw_angle += (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a - rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d) * 0.2;
    //底盘
    if (servo_cmd_send.pitch_angle > 180) {
        servo_cmd_send.pitch_angle = 180;
    } else if (servo_cmd_send.pitch_angle < 0) {
        servo_cmd_send.pitch_angle = 0;
    }
    if (servo_cmd_send.yaw_angle > 21) {
        servo_cmd_send.yaw_angle = 21;
    } else if (servo_cmd_send.yaw_angle < -179) {
        servo_cmd_send.yaw_angle = -179;
    }
    if (rc_data[TEMP].key[KEY_PRESS].w || rc_data[TEMP].key[KEY_PRESS].s) {
        chassis_cmd_send.vy = (rc_data[TEMP].key[KEY_PRESS].w * 660 * 28 - rc_data[TEMP].key[KEY_PRESS].s * 660 * 28) * ramp_calc(&chassis_vy_ramp);
    } 
    else {
        ramp_init(&chassis_vy_ramp, ACCLE_RAMP_TIME);
        if(chassis_cmd_send.vy > 0)
        {
            chassis_cmd_send.vy -= DECELE_RAMP_TIME;
        }
        else if(chassis_cmd_send.vy < 0)
        {
            chassis_cmd_send.vy += DECELE_RAMP_TIME;
        }
    }
    if (rc_data[TEMP].key[KEY_PRESS].a || rc_data[TEMP].key[KEY_PRESS].d) {
        chassis_cmd_send.vx = (rc_data[TEMP].key[KEY_PRESS].d * 660 * 20 - rc_data[TEMP].key[KEY_PRESS].a * 660 * 20) * ramp_calc(&chassis_vx_ramp);
    } else {
        ramp_init(&chassis_vx_ramp, ACCLE_RAMP_TIME);
        if(chassis_cmd_send.vx > 0)
        {
            chassis_cmd_send.vx -= DECELE_RAMP_TIME;
        }
        else if(chassis_cmd_send.vx < 0)
        {
            chassis_cmd_send.vx += DECELE_RAMP_TIME;
        }
    }
    if (rc_data[TEMP].key[KEY_PRESS].q || rc_data[TEMP].key[KEY_PRESS].e) {
        chassis_cmd_send.wz = (-rc_data[TEMP].key[KEY_PRESS].q * 660 * 12 +rc_data[TEMP].key[KEY_PRESS].e * 660 * 12) * ramp_calc(&chassis_vw_ramp);
    } else {
        ramp_init(&chassis_vw_ramp, ACCLE_RAMP_TIME);
        if(chassis_cmd_send.wz > 0)
        {
            chassis_cmd_send.wz -= 30;  
        }
        else if(chassis_cmd_send.wz < 0)
        {
            chassis_cmd_send.wz += 30;
        }
    }
    // //伸出
    // stretch_cmd_send.left_angle += rc_data[TEMP].key[KEY_PRESS].x * 16 - rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].x * 16;
    // stretch_cmd_send.right_angle -= rc_data[TEMP].key[KEY_PRESS].x * 16 - rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].x * 16;
    // //升降
    // if(rc_data[TEMP].key[KEY_PRESS].c )
    // {
    //     lift_cmd_send.left_angle = lift_fetch_data.now_left_angle + 200 * ramp_calc(&lift_l_ramp);
    //     lift_cmd_send.right_angle = lift_fetch_data.now_right_angle - 200 * ramp_calc(&lift_r_ramp);
    // }
    // else if (rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].c)
    // {
    //     lift_cmd_send.left_angle = lift_fetch_data.now_left_angle - 200 * ramp_calc(&lift_l_ramp);
    //     lift_cmd_send.right_angle = lift_fetch_data.now_right_angle + 200 * ramp_calc(&lift_r_ramp);
    // }
    // else{
    //     ramp_init(&lift_l_ramp, ACCLE_RAMP_TIME);
    //     ramp_init(&lift_r_ramp, ACCLE_RAMP_TIME);
    // }

       
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].r) {
        // 重新上电达妙板子
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
}

/**
 * @brief 气路控制
 * @attention PC10引脚是大气泵 这个好使
 * @attention PE1引脚是大气泵阀
 */
void air_controll()
{

    // 大气泵 启停 应当注意非左上右下模式，会与roll产生干扰
    // ui_cmd_send.main_air_flag为1是开 为2是关
    if ((rc_data[TEMP].key[KEY_PRESS].v || (rc_data[TEMP].rc.dial > 250)|| (Vision_Joint_Data_fetch_data.air_pump == 2))) {
        ui_cmd_send.main_air_flag = 1;
    } else if ((rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].v || rc_data[TEMP].rc.dial < -250||(Vision_Joint_Data_fetch_data.air_pump == 3)) ) {
        ui_cmd_send.main_air_flag = 2;
    }
   
    if (!((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left))) {
        // 大气泵
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, ui_cmd_send.main_air_flag == 1 ? 1 : 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1 ? 0 : 1);
    }
        
}

void Vision_Control()
{
    //暂时是遥控器决定启停
    if(((switch_is_mid(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left)))
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
    }
    if (rc_data[TEMP].key[KEY_PRESS].w || rc_data[TEMP].key[KEY_PRESS].s) {
        chassis_cmd_send.vy = (rc_data[TEMP].key[KEY_PRESS].w * 660 * 28 - rc_data[TEMP].key[KEY_PRESS].s * 660 * 28) * ramp_calc(&chassis_vy_ramp);
    } 
    else {
        ramp_init(&chassis_vy_ramp, ACCLE_RAMP_TIME);
        if(chassis_cmd_send.vy > 0)
        {
            chassis_cmd_send.vy -= DECELE_RAMP_TIME;
        }
        else if(chassis_cmd_send.vy < 0)
        {
            chassis_cmd_send.vy += DECELE_RAMP_TIME;
        }
    }
    if (rc_data[TEMP].key[KEY_PRESS].a || rc_data[TEMP].key[KEY_PRESS].d) {
        chassis_cmd_send.vx = (rc_data[TEMP].key[KEY_PRESS].d * 660 * 20 - rc_data[TEMP].key[KEY_PRESS].a * 660 * 20) * ramp_calc(&chassis_vx_ramp);
    } else {
        ramp_init(&chassis_vx_ramp, ACCLE_RAMP_TIME);
        if(chassis_cmd_send.vx > 0)
        {
            chassis_cmd_send.vx -= DECELE_RAMP_TIME;
        }
        else if(chassis_cmd_send.vx < 0)
        {
            chassis_cmd_send.vx += DECELE_RAMP_TIME;
        }
    }
    if (rc_data[TEMP].key[KEY_PRESS].q || rc_data[TEMP].key[KEY_PRESS].e) {
        chassis_cmd_send.wz = (-rc_data[TEMP].key[KEY_PRESS].q * 660 * 12 +rc_data[TEMP].key[KEY_PRESS].e * 660 * 12) * ramp_calc(&chassis_vw_ramp);
    } else {
        ramp_init(&chassis_vw_ramp, ACCLE_RAMP_TIME);
        if(chassis_cmd_send.wz > 0)
        {
            chassis_cmd_send.wz -= 30;  
        }
        else if(chassis_cmd_send.wz < 0)
        {
            chassis_cmd_send.wz += 30;
        }
    }
    if(Vision_Joint_Data_fetch_data.custom_controller_comm_recv != 0xff)
    {
        //这里加上UI反馈
    }
    
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
        //暂时是定死的，会改的
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
    SubGetMessage(ui_feed_sub, (void *)&ui_fetch_data);
    SubGetMessage(BigPitch_feed_sub, (void *)&BigPitch_fetch_data);
    SubGetMessage(YAW_feed_sub, (void *)&YAW_fetch_data);
    SubGetMessage(BIG_ROLL_feed_sub, (void *)&BIG_ROLL_fetch_data);
    SubGetMessage(SMALL_PITCH_feed_sub, (void *)&SMALL_PITCH_fetch_data);
    SubGetMessage(SMALL_ROLL_feed_sub, (void *)&SMALL_ROLL_fetch_data);
    SubGetMessage(vision_joint_data_sub, (void *)&Vision_Joint_Data_fetch_data);

    Emergency_Handling();

    if ((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left))
    {
    MouseKeySet();
    }
    else{
    RemoteControlSet();
    }

    Vision_Control();
    Multi_Joint_Motion_Solution();

    cmd_value_limit();
    Stall_Detection();

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


}