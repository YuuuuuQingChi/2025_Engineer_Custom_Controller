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
#include "forward.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "math.h"

// 以后这块搞点宏
#define PITCH_RUN_MODE 2
#define ROLL_RUN_MODE  3
#define STOP_MODE      4
#define Rotation_Ratio 1.5
#define PI             3.1415
/* cmd应用包含的模块实例指针和交互信息存储*/

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回

static Publisher_t *first_stretch_cmd_pub;                   // 一级控制消息发布者
static Subscriber_t *first_stretch_feed_sub;                 // 一级反馈信息订阅者
static First_Stretch_Ctrl_Cmd_s first_stretch_cmd_send;      // 传递给一级的控制信息
static First_Stretch_Upload_Data_s first_stretch_fetch_data; // 从一级获取的反馈信息

static Publisher_t *second_stretch_cmd_pub;                    // 二级控制消息发布者
static Subscriber_t *second_stretch_feed_sub;                  // 二级反馈信息订阅者
static Second_Stretch_Ctrl_Cmd_s second_stretch_cmd_send;      // 传递给二级的控制信息
static Second_Stretch_Upload_Data_s second_stretch_fetch_data; // 从二级获取的反馈信息

static Publisher_t *lift_cmd_pub;
static Subscriber_t *lift_feed_sub;
static Lift_Ctrl_Cmd_s lift_cmd_send;
static Lift_Upload_Data_s lift_fetch_data;

static Publisher_t *horizontal_cmd_pub;
static Subscriber_t *horizontal_feed_sub;
static Horizontal_Ctrl_Cmd_s horizontal_cmd_send;
static Horizontal_Upload_Data_s horizontal_fetch_data;

static Publisher_t *forward_cmd_pub;
static Subscriber_t *forward_feed_sub;
static Forward_Ctrl_Cmd_s forward_cmd_send;
static Forward_Upload_Data_s forward_fetch_data;

static Publisher_t *servo_cmd_pub;
static Subscriber_t *servo_feed_sub;
static Servo_Cmd_s servo_cmd_send;
static Servo_Upload_Data_s servo_fetch_data;

static Publisher_t *ui_cmd_pub;
static Subscriber_t *ui_feed_sub;
static ui_Cmd_s ui_cmd_send;
static ui_Upload_Data_s ui_fetch_data;

static Robot_Status_e robot_state; // 机器人整体工作状态

float last_angle; // pitch的最后一次编码器角度
extern int mouse_count_r;
/**
 * @brief CMD初始化/
 *
 */
void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

    lift_cmd_pub            = PubRegister("lift_cmd", sizeof(Lift_Ctrl_Cmd_s));
    lift_feed_sub           = SubRegister("lift_feed", sizeof(Lift_Upload_Data_s));
    first_stretch_cmd_pub   = PubRegister("first_stretch_cmd", sizeof(First_Stretch_Ctrl_Cmd_s));
    first_stretch_feed_sub  = SubRegister("first_stretch_feed", sizeof(First_Stretch_Upload_Data_s));
    second_stretch_cmd_pub  = PubRegister("second_stretch_cmd", sizeof(Second_Stretch_Ctrl_Cmd_s));
    second_stretch_feed_sub = SubRegister("second_stretch_feed", sizeof(Second_Stretch_Upload_Data_s));
    chassis_cmd_pub         = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub        = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    forward_cmd_pub         = PubRegister("forward_cmd", sizeof(Forward_Ctrl_Cmd_s));
    forward_feed_sub        = SubRegister("forward_feed", sizeof(Forward_Upload_Data_s));
    horizontal_cmd_pub      = PubRegister("Horizontal_cmd", sizeof(Horizontal_Ctrl_Cmd_s));
    horizontal_feed_sub     = SubRegister("Horizontal_feed", sizeof(Horizontal_Upload_Data_s));

    servo_cmd_pub  = PubRegister("servo_cmd", sizeof(Servo_Cmd_s));
    servo_feed_sub = SubRegister("servo_feed", sizeof(Servo_Upload_Data_s));

    ui_cmd_pub  = PubRegister("ui_cmd", sizeof(ui_Cmd_s));
    ui_feed_sub = SubRegister("ui_feed", sizeof(ui_Upload_Data_s));

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
    last_angle  = forward_fetch_data.new_left_angle;
}

/**
 * @brief 初始化姿态函数
 *
 */
void Init_Value()
{
}

/**
 * @brief 限位函数
 *
 */
float Limit_Set(float obj, float max, float min)
{
    if (obj > max) {
        obj = max;
    }
    if (obj < min) {
        obj = min;
    }
    return obj;
}

/**
 * @brief 限位函数
 *  全局限位：自动模式、键鼠、遥控器通用
 */
void cmd_value_limit()
{

    lift_cmd_send.left_now   = Limit_Set(lift_cmd_send.left_now, LIFT_MAX_ANGLE_LEFT, LIFT_MIN_ANGLE_LEFT);
    lift_cmd_send.right_now  = Limit_Set(lift_cmd_send.right_now, LIFT_MAX_ANGLE_RIGHT, LIFT_MIN_ANGLE_RIGHT);
    lift_cmd_send.left_last  = Limit_Set(lift_cmd_send.left_now, LIFT_MAX_ANGLE_LEFT, LIFT_MIN_ANGLE_LEFT);
    lift_cmd_send.right_last = Limit_Set(lift_cmd_send.right_now, LIFT_MAX_ANGLE_RIGHT, LIFT_MIN_ANGLE_RIGHT);

    first_stretch_cmd_send.left_now   = Limit_Set(first_stretch_cmd_send.left_now, 20500, -19000);
    first_stretch_cmd_send.right_now  = Limit_Set(first_stretch_cmd_send.right_now, STRETCH_1_MAX_ANGLE_RIGHT, STRETCH_1_MIN_ANGLE_RIGHT);
    first_stretch_cmd_send.left_last  = Limit_Set(first_stretch_cmd_send.left_now, 20500, -19000);
    first_stretch_cmd_send.right_last = Limit_Set(first_stretch_cmd_send.right_now, STRETCH_1_MAX_ANGLE_RIGHT, STRETCH_1_MIN_ANGLE_RIGHT);

    second_stretch_cmd_send.left_now   = Limit_Set(second_stretch_cmd_send.left_now, STRETCH_2_MAX_ANGLE_LEFT, STRETCH_2_MIN_ANGLE_LEFT);
    second_stretch_cmd_send.right_now  = Limit_Set(second_stretch_cmd_send.right_now, STRETCH_2_MAX_ANGLE_RIGHT, STRETCH_2_MIN_ANGLE_RIGHT);
    second_stretch_cmd_send.left_last  = Limit_Set(second_stretch_cmd_send.left_now, STRETCH_2_MAX_ANGLE_LEFT, STRETCH_2_MIN_ANGLE_LEFT);
    second_stretch_cmd_send.right_last = Limit_Set(second_stretch_cmd_send.right_now, STRETCH_2_MAX_ANGLE_RIGHT, STRETCH_2_MIN_ANGLE_RIGHT);

    horizontal_cmd_send.Now_MechAngle = Limit_Set(horizontal_cmd_send.Now_MechAngle, HORIZONTAL_MAX, HORIZONTAL_MIN);
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
int value = 0;
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
void control_forward(int16_t ch1, int16_t ch2);
void Maintain_current_posture(); // 保持当前姿态的函数，不止自动模式可以用，其他地方也可以

static void RemoteControlSet()
{
    // 左侧开关状态[上],右侧开关状态[上]
    if ((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left)) {
        chassis_cmd_send.vx = rc_data[TEMP].rc.rocker_l_ * 5*(7-lift_cmd_send.left_now/4000.0);
        chassis_cmd_send.vy = rc_data[TEMP].rc.rocker_l1 * 2*(7-lift_cmd_send.left_now/4000.0);
        chassis_cmd_send.wz = rc_data[TEMP].rc.rocker_r_ * 15;

        if (is_range(rc_data[TEMP].rc.rocker_r1)) {
            lift_cmd_send.left_now += rc_data[TEMP].rc.rocker_r1 / 20.0;
            lift_cmd_send.right_now -= rc_data[TEMP].rc.rocker_r1 / 20.0;
        }
    }
    // 左侧开关状态[中],右侧开关状态[中]
    if ((switch_is_mid(rc_data[TEMP].rc.switch_right)) && switch_is_mid(rc_data[TEMP].rc.switch_left)) {
        // 一级伸出
        if (1 - is_range(rc_data[TEMP].rc.rocker_l_) && (is_range(rc_data[TEMP].rc.rocker_r1))) {
            first_stretch_cmd_send.left_now -= rc_data[TEMP].rc.rocker_r1 / 10.0;
            first_stretch_cmd_send.right_now += rc_data[TEMP].rc.rocker_r1 / 10.0;
        }
        // 二级伸出
        if (is_range(rc_data[TEMP].rc.rocker_l1)) {
            second_stretch_cmd_send.left_now += rc_data[TEMP].rc.rocker_l1 / 20.0;
            second_stretch_cmd_send.right_now -= rc_data[TEMP].rc.rocker_l1 / 20.0;
        }
        if (1 - is_range(rc_data[TEMP].rc.rocker_r1) && (is_range(rc_data[TEMP].rc.rocker_l_))) {

            first_stretch_cmd_send.left_now -= rc_data[TEMP].rc.rocker_l_ / 10.0;
            first_stretch_cmd_send.right_now -= rc_data[TEMP].rc.rocker_l_ / 10.0;
        }
    }

    // 左侧开关状态[中],右侧开关状态[上]
    if ((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_mid(rc_data[TEMP].rc.switch_left)) {
        // 横移
        if (is_range(rc_data[TEMP].rc.rocker_r_)) {
            horizontal_cmd_send.Now_MechAngle += rc_data[TEMP].rc.rocker_r_ / 60.0;
        }
        // 前端
        control_forward(rc_data[TEMP].rc.rocker_l_ / 660.0 * 120, rc_data[TEMP].rc.rocker_r1 / 660.0 * 120);
    }

    // 左侧开关状态[中],右侧开关状态[下]
    if ((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_mid(rc_data[TEMP].rc.switch_left)) {

        servo_cmd_send.pitch_now_angle = rc_data[TEMP].rc.rocker_r1 / 660.0 * 180;
        servo_cmd_send.yaw_now_angle   = rc_data[TEMP].rc.rocker_l_ / 660.0 * 90;
        if (servo_cmd_send.pitch_now_angle > 180) {
            servo_cmd_send.pitch_now_angle = 180;
        } else if (servo_cmd_send.pitch_now_angle < -180) {
            servo_cmd_send.pitch_now_angle = -180;
        }
        if (servo_cmd_send.yaw_now_angle > 21) {
            servo_cmd_send.yaw_now_angle = 21;
        } else if (servo_cmd_send.yaw_now_angle < -179) {
            servo_cmd_send.yaw_now_angle = -179;
        }
    }
    // 双下
    if ((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left)) {
        lift_cmd_send.lift_mode                     = LIFT_STOP;
        second_stretch_cmd_send.second_stretch_mode = SECOND_STOP;
        first_stretch_cmd_send.first_stretch_mode   = FIRST_STOP;
        forward_cmd_send.Forward_mode               = FORWARD_STOP;
        horizontal_cmd_send.Horizontal_mode         = HORIZONTAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode               = CHASSIS_ZERO_FORCE;

        if (is_range(rc_data[TEMP].rc.dial)) {
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
    } else {
        chassis_cmd_send.chassis_mode               = CHASSIS_WALK;
        lift_cmd_send.lift_mode                     = LIFT;
        second_stretch_cmd_send.second_stretch_mode = SECOND_STRETCH;
        first_stretch_cmd_send.first_stretch_mode   = FIRST_YAW;
        forward_cmd_send.Forward_mode               = FORWARD_INIT_PITCH;
        horizontal_cmd_send.Horizontal_mode         = HORIZONTAL_MOVE;
    }
}

/** @todo 每个部位的复位
 * @brief 键盘模式控制
 *
 */

void PC_Mode_Set(PC_Mode_t *mode)
{
    *mode = rc_data[TEMP].mouse.count;
    // 以上是四种大模式的判断
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    PC_Mode_Set(&ui_cmd_send.PC_Mode);

    ui_cmd_send.flag_refresh_ui = ui_fetch_data.flag_refresh_ui;

    servo_cmd_send.pitch_now_angle += (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w - rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s) * 0.5;
    servo_cmd_send.yaw_now_angle += (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a - rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d) * 0.2;

    if (servo_cmd_send.pitch_now_angle > 180) {
        servo_cmd_send.pitch_now_angle = 180;
    } else if (servo_cmd_send.pitch_now_angle < 0) {
        servo_cmd_send.pitch_now_angle = 0;
    }
    if (servo_cmd_send.yaw_now_angle > 21) {
        servo_cmd_send.yaw_now_angle = 21;
    } else if (servo_cmd_send.yaw_now_angle < -179) {
        servo_cmd_send.yaw_now_angle = -179;
    }

    if (rc_data[TEMP].key[KEY_PRESS].r) ui_cmd_send.flag_refresh_ui = 1;


        chassis_cmd_send.chassis_mode = CHASSIS_WALK;
        //
        if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].w || rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s) {
            chassis_cmd_send.vy = (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].w * 660 * 5*(7-lift_cmd_send.left_now/4000.0) - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s * 660 * 5*(7-lift_cmd_send.left_now/4000.0)) * ramp_calc(&chassis_vy_ramp);
        } else {
            ramp_init(&chassis_vy_ramp, RAMP_TIME);
            chassis_cmd_send.vy = (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].w * 660 * 5*(7-lift_cmd_send.left_now/4000.0) - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s * 660 * 2*(7-lift_cmd_send.left_now/4000.0)) * ramp_calc(&chassis_vy_ramp);
        }
        //
        if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].a || rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d) {
            chassis_cmd_send.vx = (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d * 660 * 2*(7-lift_cmd_send.left_now/4000.0) - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].a * 660 * 15) * ramp_calc(&chassis_vx_ramp);
        } else {
            ramp_init(&chassis_vx_ramp, RAMP_TIME);
            chassis_cmd_send.vx = (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d * 660 * 5*(7-lift_cmd_send.left_now/4000.0) - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].a * 660 * 15) * ramp_calc(&chassis_vx_ramp);
        }

        if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q || rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e) {
            chassis_cmd_send.wz = (-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q * 660 * 8 +rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e * 660 * 8) * ramp_calc(&chassis_vw_ramp);
        } else {
            ramp_init(&chassis_vw_ramp, RAMP_TIME);
            chassis_cmd_send.wz = (-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q * 660 * 8 + rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e * 660 * 8) * ramp_calc(&chassis_vw_ramp);
        }
        first_stretch_cmd_send.left_now += -rc_data[TEMP].key[KEY_PRESS].z * 50 + rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].z * 50 - rc_data[TEMP].key[KEY_PRESS].s * 50 + rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].s * 50;
        first_stretch_cmd_send.right_now += -rc_data[TEMP].key[KEY_PRESS].z * 50 + rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].z * 50 + rc_data[TEMP].key[KEY_PRESS].s * 50 - rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].s * 50;

        second_stretch_cmd_send.left_now += rc_data[TEMP].key[KEY_PRESS].x * 16 - rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].x * 16;
        second_stretch_cmd_send.right_now -= rc_data[TEMP].key[KEY_PRESS].x * 16 - rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].x * 16;

        lift_cmd_send.left_now += (rc_data[TEMP].key[KEY_PRESS].c * 28 - rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].c * 28);
        lift_cmd_send.right_now -= (rc_data[TEMP].key[KEY_PRESS].c * 28 - rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].c * 28);

        horizontal_cmd_send.Now_MechAngle += (rc_data[TEMP].key[KEY_PRESS].d * 11 - rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].d * 11);

        control_forward((rc_data[TEMP].key[KEY_PRESS].q * 40 - rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].q * 40), (rc_data[TEMP].key[KEY_PRESS].e * 80 - rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].e * 80));
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].r) {
        // 重新上电达妙板子
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
}

/**
 * @brief 前端的控制逻辑
 *
 *
 */

int8_t mode; // pitch和roll的模式
int8_t last_mode;

float relevant_angle;  // pitch和roll的相对角度
float final_angle;     // 目标角度
float roll_last_angle; // roll的最后一次编码器角度
void mode_change();    // 前端模式的选择函数
void control_forward(int16_t ch1, int16_t ch2)
{
    last_mode = mode;
    mode_change();
    if (mode == PITCH_RUN_MODE) // pitch
    {
        last_angle = forward_fetch_data.new_left_angle;
    }
    if (mode == ROLL_RUN_MODE) // roll
    {
        relevant_angle = (forward_fetch_data.new_forward_angle - roll_last_angle) * Rotation_Ratio;
        last_angle     = forward_fetch_data.new_left_angle;
    }
    if (mode != ROLL_RUN_MODE && last_mode == ROLL_RUN_MODE) {
        relevant_angle = 0;
    }
    if (mode != ROLL_RUN_MODE) {
        roll_last_angle = forward_fetch_data.new_forward_angle;
    }
    final_angle                   = ch1 + ch2 + last_angle + relevant_angle;
    forward_cmd_send.angel_output = PIDCalculate(encoder_pid, forward_fetch_data.new_left_angle, final_angle);

    if (mode == ROLL_RUN_MODE) {
        forward_cmd_send.angel_output1 = -(forward_cmd_send.angel_output);
    } else {
        forward_cmd_send.angel_output1 = forward_cmd_send.angel_output;
    }
}

void mode_change()
{

    if ((!is_range(rc_data[TEMP].rc.rocker_l_) && is_range(rc_data[TEMP].rc.rocker_r1)) || (((rc_data[TEMP].key[KEY_PRESS].q) || (rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].q)) && (!(rc_data[TEMP].key[KEY_PRESS].e) || !(rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].e)))) {
        mode                          = PITCH_RUN_MODE;
        forward_cmd_send.Forward_mode = PITCH;
    } else if ((is_range(rc_data[TEMP].rc.rocker_l_) && !is_range(rc_data[TEMP].rc.rocker_r1)) || ((!(rc_data[TEMP].key[KEY_PRESS].q) || !(rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].q)) && ((rc_data[TEMP].key[KEY_PRESS].e) || (rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].e)))) {
        mode                          = ROLL_RUN_MODE;
        forward_cmd_send.Forward_mode = ROLL;
    } else {
        mode                          = STOP_MODE;
        forward_cmd_send.Forward_mode = PITCH;
    }
}

/**
 * @brief 气路控制
 * @attention PB8引脚是左气泵
 * @attention PB9引脚是右气泵
 * @attention PC10引脚是大气泵 这个好使
 * @attention PD14引脚是上气缸阀
 * @attention PD15引脚是左气泵阀
 * @attention PE0引脚是右气泵阀
 * @attention PE1引脚是大气泵阀
 */
void air_controll()
{

    // 大气泵
    if (rc_data[TEMP].key[KEY_PRESS].v || (rc_data[TEMP].rc.dial > 250)) {
        ui_cmd_send.main_air_flag = 1;
    } else if (rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].v || rc_data[TEMP].rc.dial < -250) {
        ui_cmd_send.main_air_flag = 2;
    }
    // 小气泵左 电脑f 遥控器右垂直 ->下吸盘
    if (rc_data[TEMP].key[KEY_PRESS].f || ((switch_is_mid(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left) && (rc_data[TEMP].rc.rocker_r1 > 200 || rc_data[TEMP].rc.rocker_r1 < -200))) {
        ui_cmd_send.left_air_flag = 1;
    } else if (rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].f || ((switch_is_mid(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left) && !is_range(rc_data[TEMP].rc.rocker_r1))) {
        ui_cmd_send.left_air_flag = 2;
    }
    // 小气泵右 上气缸 电脑g 遥控器左垂直 ->上吸盘
    if (rc_data[TEMP].key[KEY_PRESS].f || ((switch_is_mid(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left) && (rc_data[TEMP].rc.rocker_l1 > 200 || rc_data[TEMP].rc.rocker_l1 < -200))) {
        ui_cmd_send.right_air_flag   = 1;
        ui_cmd_send.air_up_gang_flag = 1;
    } else if (rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].f || ((switch_is_mid(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left) && !is_range(rc_data[TEMP].rc.rocker_l1))) {
        ui_cmd_send.air_up_gang_flag = 2;
        ui_cmd_send.right_air_flag   = 2;
    }
    // 下气缸 电脑b 遥控器右水平
    if (rc_data[TEMP].key[KEY_PRESS].b || ((switch_is_mid(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left) && (rc_data[TEMP].rc.rocker_r_ > 200 || rc_data[TEMP].rc.rocker_r_ < -200))) {
        ui_cmd_send.air_down_gang_flag = 1;
    } else if (rc_data[TEMP].key[KEY_PRESS_MOUSE_LEFT].b || ((switch_is_mid(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left) && !is_range(rc_data[TEMP].rc.rocker_r_))) {
        ui_cmd_send.air_down_gang_flag = 2;
    }

    if (!((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left))) {
        // 大气泵
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, ui_cmd_send.main_air_flag == 1 ? 1 : 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1 ? 0 : 1);
        // 小气泵右 上气缸 电脑g 遥控器左垂直 ->上吸盘
        // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,ui_cmd_send.right_air_flag == 1 ? 1:0);
        // HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,ui_cmd_send.air_up_gang_flag == 1 ? 1:0);
        // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14) == 1 ? 0:1);
        // 下气缸 电脑b 遥控器右水平
        // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,ui_cmd_send.air_down_gang_flag == 1 ? 1:0);
        // 小气泵左 电脑f 遥控器右垂直 ->下吸盘
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, ui_cmd_send.left_air_flag == 1 ? 1 : 0);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 1 ? 0 : 1);

        //
    }
}

/**
 * @brief 自动模式控制
 *
 */
int16_t point;
int16_t flag_r1, flag_r2, flag_r3, flag_r4; // 数据小心会溢出
int16_t flag1 = 1, flag2, flag3 = 1, flag4, flag5 = 1, flag6;
void auto_mode_decide();                                                                     // 自动模式选择函数
void Put_it_back_in_the_silo();                                                              // 放回矿仓
void auto_small_resource_island();                                                           // 取小资源岛
float Automatic_mode_target_setting(float target, float measure, float expected_increments); // 自动模式目标值设定函数，防止一次给的值过于太大了，导致疯车

void auto_mode() // 自动模式最终函数
{
    auto_mode_decide();
    if (rc_data[TEMP].rc.rocker_l_ > 200 || rc_data[TEMP].rc.rocker_l_ < -200) {
        ui_cmd_send.auto_confirm_flag = 1;
        auto_small_resource_island(); // 取小资源岛
        Put_it_back_in_the_silo();    // 扔矿仓
    } else {
        ui_cmd_send.auto_confirm_flag = 0;
        flag3                         = 1;
        flag5                         = 1;
        Maintain_current_posture(); // 维持当前姿态
    }
}
void auto_mode_decide()
{

    if (rc_data[TEMP].rc.rocker_r_ >= 200 && (!is_range(rc_data[TEMP].rc.rocker_l_))) {
        flag_r1++;
    } else if (!is_range(rc_data[TEMP].rc.rocker_r_)) {
        flag_r1 = 0;
    }
    if (rc_data[TEMP].rc.rocker_r_ <= -200 && (!is_range(rc_data[TEMP].rc.rocker_l_))) {
        flag_r2++;
    } else if (!is_range(rc_data[TEMP].rc.rocker_r_)) {
        flag_r2 = 0;
    }
    if (rc_data[TEMP].rc.rocker_r1 >= 200 && (!is_range(rc_data[TEMP].rc.rocker_l_))) {
        flag_r3++;
    } else if (!is_range(rc_data[TEMP].rc.rocker_r1)) {
        flag_r3 = 0;
    }
    if (rc_data[TEMP].rc.rocker_r1 <= -200 && (!is_range(rc_data[TEMP].rc.rocker_l_))) {
        flag_r4++;
    } else if (!is_range(rc_data[TEMP].rc.rocker_r1)) {
        flag_r4 = 0;
    }
    if (flag_r1 > 100) {
        ui_cmd_send.auto_decide_flag = 1; // 左矿

    } else if (flag_r2 > 100) {
        ui_cmd_send.auto_decide_flag = 2; // 中矿
    } else if (flag_r3 > 100) {
        ui_cmd_send.auto_decide_flag = 3; // 右矿

    } else if (flag_r4 > 100) {
        ui_cmd_send.auto_decide_flag = 4; // 收回矿仓
    }
}
void auto_small_resource_island() // 取小资源岛
{
    if (ui_cmd_send.auto_decide_flag == 1 || ui_cmd_send.auto_decide_flag == 2 || ui_cmd_send.auto_decide_flag == 3) {
        if (flag3 == 1) {
            lift_cmd_send.left_now  = Automatic_mode_target_setting(16229, lift_fetch_data.new_left_angle, 100);
            lift_cmd_send.right_now = Automatic_mode_target_setting(-15907, lift_fetch_data.new_right_angle, 100);
            if ((lift_fetch_data.new_left_angle > 15800 && lift_fetch_data.new_left_angle < 16700) && (lift_fetch_data.new_right_angle < -15200 && lift_fetch_data.new_right_angle > -16500)) {
                flag1 = 1;
            } else {
                flag1 = 0;
            }
        }
        if (flag1 == 1 || ((lift_fetch_data.new_left_angle > 15800 && lift_fetch_data.new_left_angle < 16700) && (lift_fetch_data.new_right_angle < -15200 && lift_fetch_data.new_right_angle > -16500))) {
            first_stretch_cmd_send.left_now  = Automatic_mode_target_setting(15416, first_stretch_fetch_data.new_left_encoder, 150);
            first_stretch_cmd_send.right_now = Automatic_mode_target_setting(-15577, first_stretch_fetch_data.new_right_encoder, 150);

            second_stretch_cmd_send.right_now = Automatic_mode_target_setting(1, second_stretch_fetch_data.new_right_angle, 50);
            second_stretch_cmd_send.left_now  = Automatic_mode_target_setting(-111, second_stretch_fetch_data.new_left_angle, 50);
            switch (ui_cmd_send.auto_decide_flag) {
                case 1: // 左矿
                    if (second_stretch_fetch_data.new_left_angle > -5900 || second_stretch_fetch_data.new_right_angle < 6300) {
                        horizontal_cmd_send.Now_MechAngle = Automatic_mode_target_setting(-13262, horizontal_fetch_data.Horizontal_Movement, 60);
                    } else {
                        horizontal_cmd_send.Now_MechAngle = horizontal_fetch_data.Horizontal_Movement;
                    }
                    if (horizontal_fetch_data.Horizontal_Movement > -14000 && horizontal_fetch_data.Horizontal_Movement < -12500) {
                        flag2 = 1;
                    } else {
                        flag2 = 0;
                    }
                    break;
                case 2: // 中矿
                    if (second_stretch_fetch_data.new_left_angle > -5900 || second_stretch_fetch_data.new_right_angle < 6300) {
                        horizontal_cmd_send.Now_MechAngle = Automatic_mode_target_setting(27, horizontal_fetch_data.Horizontal_Movement, 60);
                    } else {
                        horizontal_cmd_send.Now_MechAngle = horizontal_fetch_data.Horizontal_Movement;
                    }
                    if (horizontal_fetch_data.Horizontal_Movement > -100 && horizontal_fetch_data.Horizontal_Movement < 100) {
                        flag2 = 1;
                    } else {
                        flag2 = 0;
                    }
                    break;
                case 3: // 右矿
                    if (second_stretch_fetch_data.new_left_angle > -5900 || second_stretch_fetch_data.new_right_angle < 6300) {
                        horizontal_cmd_send.Now_MechAngle = Automatic_mode_target_setting(11696, horizontal_fetch_data.Horizontal_Movement, 60);
                    } else {
                        horizontal_cmd_send.Now_MechAngle = horizontal_fetch_data.Horizontal_Movement;
                    }
                    if (horizontal_fetch_data.Horizontal_Movement > 10900 && horizontal_fetch_data.Horizontal_Movement < 12000) {
                        flag2 = 1;
                    } else {
                        flag2 = 0;
                    }
                    break;
                default: // 以防autofalg出现问题
                    horizontal_cmd_send.Now_MechAngle = horizontal_fetch_data.Horizontal_Movement;
                    break;
            }
            if (first_stretch_fetch_data.new_left_encoder > 15000 && first_stretch_fetch_data.new_left_encoder < 15500 && second_stretch_fetch_data.new_left_angle < 120 && second_stretch_fetch_data.new_left_angle > -200 && flag2 == 1) {
                lift_cmd_send.left_now  = Automatic_mode_target_setting(14172, lift_fetch_data.new_left_angle, 100);
                lift_cmd_send.right_now = Automatic_mode_target_setting(-14167, lift_fetch_data.new_right_angle, 100);
                flag3                   = 0;
            } else {
                flag3 = 1;
            }
        }
    }
}
// 放回矿仓
void Put_it_back_in_the_silo()
{
    if (ui_cmd_send.auto_decide_flag == 4) {
        if (flag5 == 1) {
            lift_cmd_send.left_now  = Automatic_mode_target_setting(18729, lift_fetch_data.new_left_angle, 100);
            lift_cmd_send.right_now = Automatic_mode_target_setting(-18407, lift_fetch_data.new_right_angle, 100);
            if ((lift_fetch_data.new_left_angle > 18520 && lift_fetch_data.new_left_angle < 18940) && (lift_fetch_data.new_right_angle > -18620 && lift_fetch_data.new_right_angle < -18240)) {
                flag4 = 1;
            } else {
                flag4 = 0;
            }
        }
        if (flag4 == 1 || ((lift_fetch_data.new_left_angle > 18520 && lift_fetch_data.new_left_angle < 18940) && (lift_fetch_data.new_right_angle > -18620 && lift_fetch_data.new_right_angle < -18240))) {
            horizontal_cmd_send.Now_MechAngle = Automatic_mode_target_setting(27, horizontal_fetch_data.Horizontal_Movement, 60);
            if (horizontal_fetch_data.Horizontal_Movement < 200 && horizontal_fetch_data.Horizontal_Movement > -200) // 横移回到了二级的里面
            {
                second_stretch_cmd_send.left_now  = Automatic_mode_target_setting(-14404, second_stretch_fetch_data.new_left_angle, 50);
                second_stretch_cmd_send.right_now = Automatic_mode_target_setting(14370, second_stretch_fetch_data.new_right_angle, 50);
                first_stretch_cmd_send.left_now   = Automatic_mode_target_setting(21703, first_stretch_fetch_data.new_left_encoder, 150);
                first_stretch_cmd_send.right_now  = Automatic_mode_target_setting(-21090, first_stretch_fetch_data.new_right_encoder, 150);
                if (first_stretch_fetch_data.new_left_encoder > 20000 && first_stretch_fetch_data.new_left_encoder < 23000 && first_stretch_fetch_data.new_right_encoder < -19300 && first_stretch_fetch_data.new_right_encoder > -23000 && second_stretch_fetch_data.new_right_angle > 12700 && second_stretch_fetch_data.new_right_angle < 15800 && second_stretch_fetch_data.new_left_angle < -13500 && second_stretch_fetch_data.new_left_angle > -16500) {
                    lift_cmd_send.left_now  = Automatic_mode_target_setting(9541, lift_fetch_data.new_left_angle, 100);
                    lift_cmd_send.right_now = Automatic_mode_target_setting(-9547, lift_fetch_data.new_right_angle, 100);
                    flag5                   = 0;
                } else {
                    flag5                   = 1;
                    lift_cmd_send.left_now  = lift_fetch_data.new_left_angle;
                    lift_cmd_send.right_now = lift_fetch_data.new_right_angle;
                }
            } else {
                first_stretch_cmd_send.left_now   = first_stretch_fetch_data.new_left_encoder;
                first_stretch_cmd_send.left_now   = first_stretch_fetch_data.new_left_encoder;
                second_stretch_cmd_send.left_now  = second_stretch_fetch_data.new_left_angle;
                second_stretch_cmd_send.right_now = second_stretch_fetch_data.new_right_angle;
                lift_cmd_send.left_now            = lift_fetch_data.new_left_angle;
                lift_cmd_send.right_now           = lift_fetch_data.new_right_angle;
            }
        }
    }
}
// 取出矿仓
void Extracting_ore()
{
    lift_cmd_send.left_now  = Automatic_mode_target_setting(18729, lift_fetch_data.new_left_angle, 100);
    lift_cmd_send.right_now = Automatic_mode_target_setting(-18407, lift_fetch_data.new_right_angle, 100);
    if (lift_fetch_data.new_left_angle > 18000 && lift_fetch_data.new_left_angle < 19300 && lift_fetch_data.new_right_angle > -19000 && lift_fetch_data.new_left_angle < -17900) {
        flag6 = 1;
    } else {
        flag6 = 2;
    }
    if (flag6 == 1 || (lift_fetch_data.new_left_angle > 18000 && lift_fetch_data.new_left_angle < 19300 && lift_fetch_data.new_right_angle > -19000 && lift_fetch_data.new_left_angle < -17900)) {
        first_stretch_cmd_send.right_now  = Automatic_mode_target_setting(4420, first_stretch_fetch_data.new_right_encoder, 150);
        first_stretch_cmd_send.left_now   = Automatic_mode_target_setting(-4584, first_stretch_fetch_data.new_left_encoder, 150);
        second_stretch_cmd_send.right_now = Automatic_mode_target_setting(1, second_stretch_fetch_data.new_right_angle, 50);
        second_stretch_cmd_send.left_now  = Automatic_mode_target_setting(-111, second_stretch_fetch_data.new_left_angle, 50);
        horizontal_cmd_send.Now_MechAngle = Automatic_mode_target_setting(27, horizontal_fetch_data.Horizontal_Movement, 60);
    } else {
        first_stretch_cmd_send.left_now   = first_stretch_fetch_data.new_left_encoder;
        first_stretch_cmd_send.right_now  = first_stretch_fetch_data.new_right_encoder;
        second_stretch_cmd_send.left_now  = second_stretch_fetch_data.new_left_angle;
        second_stretch_cmd_send.right_now = second_stretch_fetch_data.new_right_angle;
        horizontal_cmd_send.Now_MechAngle = horizontal_fetch_data.Horizontal_Movement;
    }
}
// 保持当前姿态的函数，不止自动模式可以用，其他地方也可以
void Maintain_current_posture()
{
    first_stretch_cmd_send.left_now   = first_stretch_fetch_data.new_left_encoder;
    first_stretch_cmd_send.right_now  = first_stretch_fetch_data.new_right_encoder;
    second_stretch_cmd_send.left_now  = second_stretch_fetch_data.new_left_angle;
    second_stretch_cmd_send.right_now = second_stretch_fetch_data.new_right_angle;
    lift_cmd_send.left_now            = lift_fetch_data.new_left_angle;
    lift_cmd_send.right_now           = lift_fetch_data.new_right_angle;
    horizontal_cmd_send.Now_MechAngle = horizontal_fetch_data.Horizontal_Movement;
}
// 自动模式目标值设定函数，防止一次给的值过于太大了，导致疯车
float Automatic_mode_target_setting(float target, float measure, float expected_increments)
{
    float control_value;
    if (measure - target < -200) {
        control_value = measure + expected_increments;
    } else if (measure - target > 200) {
        control_value = measure - expected_increments;
    } else {
        control_value = target;
    }
    return control_value;
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
    SubGetMessage(first_stretch_feed_sub, (void *)&first_stretch_fetch_data);
    SubGetMessage(second_stretch_feed_sub, (void *)&second_stretch_fetch_data);
    SubGetMessage(forward_feed_sub, (void *)&forward_fetch_data);
    SubGetMessage(horizontal_feed_sub, (void *)&horizontal_fetch_data);
    SubGetMessage(servo_feed_sub, (void *)&servo_fetch_data);
    SubGetMessage(ui_feed_sub, (void *)&ui_fetch_data);
    // 遥控器左下右上，切换为电脑模式
    if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_up(rc_data[TEMP].rc.switch_right)) {

        MouseKeySet();

    } else {
        RemoteControlSet();
    }
    // 遥控器左下右中 || 电脑切换成自动模式
    if (ui_cmd_send.PC_Mode == PC_To_AUTO_MODE || ((switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_mid(rc_data[TEMP].rc.switch_right)))) {
        auto_mode();
    }
    air_controll();
    cmd_value_limit();
    PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(lift_cmd_pub, (void *)&lift_cmd_send);
    PubPushMessage(first_stretch_cmd_pub, (void *)&first_stretch_cmd_send);
    PubPushMessage(second_stretch_cmd_pub, (void *)&second_stretch_cmd_send);
    PubPushMessage(forward_cmd_pub, (void *)&forward_cmd_send);
    PubPushMessage(horizontal_cmd_pub, (void *)&horizontal_cmd_send);
    PubPushMessage(servo_cmd_pub, (void *)&servo_cmd_send);
}