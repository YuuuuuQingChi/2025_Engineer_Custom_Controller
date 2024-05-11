// app
#include "robot_def.h"
#include "robot_cmd.h"
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

static Publisher_t *lift_cmd_pub;          // 升降控制消息发布者
static Subscriber_t *lift_feed_sub;        // 升降反馈信息订阅者
static Lift_Ctrl_Cmd_s lift_cmd_send;      // 传递给升降的控制信息
static Lift_Upload_Data_s lift_fetch_data; // 从升降获取的反馈信息

static Publisher_t *horizontal_cmd_pub;                // 横移控制消息发布者
static Subscriber_t *horizontal_feed_sub;              // 横移反馈信息订阅者
static Horizontal_Ctrl_Cmd_s horizontal_cmd_send;      // 传递给横移的控制信息
static Horizontal_Upload_Data_s horizontal_fetch_data; // 从横移获取的反馈信息

static Publisher_t *forward_cmd_pub;             // 前端控制消息发布者
static Subscriber_t *forward_feed_sub;           // 前端反馈信息订阅者
static Forward_Ctrl_Cmd_s forward_cmd_send;      // 传递给前端的控制信息
static Forward_Upload_Data_s forward_fetch_data; // 从前端获取的反馈信息

static Publisher_t *servo_cmd_pub;           // 升降控制消息发布者
static Subscriber_t *servo_feed_sub;         // 升降反馈信息订阅者
static Servo_Cmd_s servo_cmd_send;           // 传递给升降的控制信息
static Servo_Upload_Data_s servo_fetch_data; // 从升降获取的反馈信息

PC_Mode_t PC_Mode;

static Robot_Status_e robot_state; // 机器人整体工作状态




/**
 * @brief CMD初始化
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

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
    // chassis_cmd_send.chassis_mode=CHASSIS_WALK;
    // first_stretch_cmd_send.first_stretch_mode=FIRST_STRETCH;
    // first_stretch_cmd_send.left_now=-STRETCH_1_INIT_ANGLE_LEFT;
    // first_stretch_cmd_send.right_now=STRETCH_1_INIT_ANGLE_LEFT;
    // first_stretch_cmd_send.left_last=-STRETCH_1_INIT_ANGLE_RIGHT;
    // first_stretch_cmd_send.right_last=STRETCH_1_INIT_ANGLE_RIGHT;

    // lift_cmd_send.lift_mode = LIFT;
    // lift_cmd_send.left_now=LIFT_INIT_ANGLE_LEFT;
    // lift_cmd_send.right_now=LIFT_INIT_ANGLE_RIGHT;
    // lift_cmd_send.left_last=LIFT_INIT_ANGLE_LEFT;
    // lift_cmd_send.right_last=LIFT_INIT_ANGLE_RIGHT;
    // Init_Value();
    // last_angle = forward_fetch_data.new_left_angle;
}




/**
 * @brief 初始化姿态函数
 *
 */
void Init_Value()
{
    // lift_cmd_send.lift_mode = LIFT;
    // chassis_cmd_send.chassis_mode=CHASSIS_WALK;

    // lift_cmd_send.left_now=LIFT_INIT_ANGLE;
    // lift_cmd_send.right_now=-LIFT_INIT_ANGLE;
    // lift_cmd_send.left_last=LIFT_INIT_ANGLE;
    // lift_cmd_send.right_last=-LIFT_INIT_ANGLE;

    // first_stretch_cmd_send.first_stretch_mode=FIRST_STRETCH;
    // first_stretch_cmd_send.left_now=-STRETCH_1_INIT_ANGLE;
    // first_stretch_cmd_send.right_now=STRETCH_1_INIT_ANGLE;
    // first_stretch_cmd_send.left_last=-STRETCH_1_INIT_ANGLE;
    // first_stretch_cmd_send.right_last=STRETCH_1_INIT_ANGLE;

    // horizontal_cmd_send.Horizontal_mode=HORIZONTAL_MOVE;
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
void control_forward(int16_t ch1, int16_t ch2);
void Maintain_current_posture(); // 保持当前姿态的函数，不止自动模式可以用，其他地方也可以
// 可能其他地方也用所以我声明在了最上面
float last_first_right_angle, last_first_left_angle;
float last_second_right_angle, last_second_left_angle;
float last_lift_right_angle, last_lift_left_angle;
float last_horizontal_angle;

static void RemoteControlSet()
{
    if ((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_up(rc_data[TEMP].rc.switch_left)) {
        if (is_range(rc_data[TEMP].rc.rocker_l1) || is_range(rc_data[TEMP].rc.rocker_l_) || is_range(rc_data[TEMP].rc.rocker_r_)) {
            chassis_cmd_send.chassis_mode = CHASSIS_WALK;
            chassis_cmd_send.vx           = rc_data[TEMP].rc.rocker_l_ * 20; // 系数待测
            chassis_cmd_send.vy           = rc_data[TEMP].rc.rocker_l1 * 20;
            chassis_cmd_send.wz           = -rc_data[TEMP].rc.rocker_r_ * 15;
        }

        if (is_range(rc_data[TEMP].rc.rocker_r1)) {
            lift_cmd_send.lift_mode = LIFT;
            lift_cmd_send.left_now += rc_data[TEMP].rc.rocker_r1 / 20.0;
            lift_cmd_send.right_now -= rc_data[TEMP].rc.rocker_r1 / 20.0;
            lift_cmd_send.left_last  = lift_cmd_send.left_now;
            lift_cmd_send.right_last = lift_cmd_send.right_now;
        }
    }
    // 右侧开关状态[中],左侧开关状态[中]
    if ((switch_is_mid(rc_data[TEMP].rc.switch_right)) && switch_is_mid(rc_data[TEMP].rc.switch_left)) {
        // 一级伸出
        if (1 - is_range(rc_data[TEMP].rc.rocker_l_) && (is_range(rc_data[TEMP].rc.rocker_r1))) {
            first_stretch_cmd_send.first_stretch_mode = FIRST_STRETCH;
            first_stretch_cmd_send.left_now -= rc_data[TEMP].rc.rocker_r1 / 10.0;
            first_stretch_cmd_send.right_now += rc_data[TEMP].rc.rocker_r1 / 10.0;
            first_stretch_cmd_send.right_last = first_stretch_cmd_send.right_now;
            first_stretch_cmd_send.left_last  = first_stretch_cmd_send.left_now;
        } else {
            first_stretch_cmd_send.right_now = first_stretch_cmd_send.right_last;
            first_stretch_cmd_send.left_now  = first_stretch_cmd_send.left_last;
        }
        // 二级伸出
        if (is_range(rc_data[TEMP].rc.rocker_l1)) {
            second_stretch_cmd_send.second_stretch_mode = SECOND_STRETCH;
            second_stretch_cmd_send.left_now += rc_data[TEMP].rc.rocker_l1 / 23.0;
            second_stretch_cmd_send.right_now -= rc_data[TEMP].rc.rocker_l1 / 23.0;
            second_stretch_cmd_send.right_last = second_stretch_cmd_send.right_now;
            second_stretch_cmd_send.left_last  = second_stretch_cmd_send.left_now;
        } else if (1 - is_range(rc_data[TEMP].rc.rocker_l1)) {
            second_stretch_cmd_send.right_now = second_stretch_cmd_send.right_last;
            second_stretch_cmd_send.left_now  = second_stretch_cmd_send.left_last;
        }
    }

    // 右侧开关状态[上],左侧开关状态[中]
    if ((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_mid(rc_data[TEMP].rc.switch_left)) {
        // 横移
        if (is_range(rc_data[TEMP].rc.rocker_r_)) {
            horizontal_cmd_send.Now_MechAngle += rc_data[TEMP].rc.rocker_r_ / 60.0;
            horizontal_cmd_send.Last_MechAngle = horizontal_cmd_send.Now_MechAngle;
        } else if (1 - is_range(rc_data[TEMP].rc.rocker_r_)) {
            horizontal_cmd_send.Now_MechAngle = horizontal_cmd_send.Last_MechAngle;
        }

        if (1 - is_range(rc_data[TEMP].rc.rocker_r1) && (is_range(rc_data[TEMP].rc.rocker_l_))) {
            first_stretch_cmd_send.first_stretch_mode = FIRST_YAW;
            first_stretch_cmd_send.left_now -= rc_data[TEMP].rc.rocker_l_ / 10.0;
            first_stretch_cmd_send.right_now -= rc_data[TEMP].rc.rocker_l_ / 10.0;
            first_stretch_cmd_send.right_last = first_stretch_cmd_send.right_now;
            first_stretch_cmd_send.left_last  = first_stretch_cmd_send.left_now;
        } else {
            first_stretch_cmd_send.right_now = first_stretch_cmd_send.right_last;
            first_stretch_cmd_send.left_now  = first_stretch_cmd_send.left_last;
        }

        if (is_range(rc_data[TEMP].rc.dial)) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);

        } else {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);
        }
        // 前端
         control_forward(rc_data[TEMP].rc.rocker_l_ / 660.0 * 120,rc_data[TEMP].rc.rocker_r1 / 660.0 * 120);
    }
    
    // 右侧开关状态[下],左侧开关状态[中]
    if ((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_mid(rc_data[TEMP].rc.switch_left)) {
        servo_cmd_send.pitch_now_angle += rc_data[TEMP].rc.rocker_r1 / 660.0 * 10;
        servo_cmd_send.yaw_now_angle += rc_data[TEMP].rc.rocker_l_ / 660.0 * 10;
    }

    // 双下
    if ((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left)) {
        lift_cmd_send.lift_mode                     = LIFT_STOP;
        second_stretch_cmd_send.second_stretch_mode = SECOND_STOP;
        first_stretch_cmd_send.first_stretch_mode   = FIRST_STOP;
        forward_cmd_send.Forward_mode = FORWARD_STOP;
        horizontal_cmd_send.Horizontal_mode = HORIZONTAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode               = CHASSIS_ZERO_FORCE;
        if (is_range(rc_data[TEMP].rc.dial)) {
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
        // 重新上电
    }

    // 差个升降的限位，上车测数据再说
    // 以上是升降的控制逻辑
    // lift_cmd_send.left_now=lift_cmd_send.left_last;
    // lift_cmd_send.right_now=lift_cmd_send.right_last;
    //  lift_cmd_send.left_now=Limit_Set(lift_cmd_send.left_now,LIFT_MAX_ANGLE_LEFT,LIFT_MIN_ANGLE_LEFT);
    //  lift_cmd_send.right_now=Limit_Set(lift_cmd_send.right_now,LIFT_MAX_ANGLE_RIGHT,LIFT_MIN_ANGLE_RIGHT);
    //  lift_cmd_send.left_last=Limit_Set(lift_cmd_send.left_now,LIFT_M  AX_ANGLE_LEFT,LIFT_MIN_ANGLE_LEFT);
    //  lift_cmd_send.right_last=Limit_Set(lift_cmd_send.right_now,LIFT_MAX_ANGLE_RIGHT,LIFT_MIN_ANGLE_RIGHT);

    // first_stretch_cmd_send.left_now=Limit_Set(first_stretch_cmd_send.left_now,24000,-38000);
    // first_stretch_cmd_send.right_now=Limit_Set(first_stretch_cmd_send.right_now,STRETCH_1_MAX_ANGLE_RIGHT,STRETCH_1_MIN_ANGLE_RIGHT);
    // first_stretch_cmd_send.left_last=Limit_Set(first_stretch_cmd_send.left_now,24000,-38000);
    // first_stretch_cmd_send.right_last=Limit_Set(first_stretch_cmd_send.right_now,STRETCH_1_MAX_ANGLE_RIGHT,STRETCH_1_MIN_ANGLE_RIGHT);

    // second_stretch_cmd_send.left_now=Limit_Set(second_stretch_cmd_send.left_now,STRETCH_2_MAX_ANGLE_LEFT,STRETCH_2_MIN_ANGLE_LEFT);
    // second_stretch_cmd_send.right_now=Limit_Set(second_stretch_cmd_send.right_now,STRETCH_2_MAX_ANGLE_RIGHT,STRETCH_2_MIN_ANGLE_RIGHT);
    // second_stretch_cmd_send.left_last=Limit_Set(second_stretch_cmd_send.left_now,STRETCH_2_MAX_ANGLE_LEFT,STRETCH_2_MIN_ANGLE_LEFT);
    // second_stretch_cmd_send.right_last=Limit_Set(second_stretch_cmd_send.right_now,STRETCH_2_MAX_ANGLE_RIGHT,STRETCH_2_MIN_ANGLE_RIGHT);

    // horizontal_cmd_send.Now_MechAngle=Limit_Set(horizontal_cmd_send.Now_MechAngle,HORIZONTAL_MAX,HORIZONTAL_MIN);
    // 气泵 除双下模式都可以用
    if (is_range(rc_data[TEMP].rc.dial) && !((switch_is_down(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left))) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);

    } else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);
    }
}




/** @todo 每个部位的复位
 * @brief 键盘模式控制
 *
 */

void PC_Mode_Set(PC_Mode_t *mode)
{
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w) {
        *mode = PC_Walk;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s) {
        *mode = PC_Get_Money;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].f) {
        *mode = PC_To_Begin_ALL;
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].r) {
        *mode = DA_MIAO_Reset_All;
    }
    // 以上是四种大模式的判断
}



int flag_refresh_ui=0;
/**
 * @brief 输入为键鼠时模式和控制量设置
 *  
 */
static void MouseKeySet()
{
    PC_Mode_Set(&PC_Mode);

    if (PC_Mode == PC_Walk) {
        // 行走模式
        if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].r)flag_refresh_ui=1;
        chassis_cmd_send.chassis_mode = CHASSIS_WALK;
        chassis_cmd_send.vy           = rc_data[TEMP].key[KEY_PRESS].w * 660 * 12 - rc_data[TEMP].key[KEY_PRESS].s * 660 * 12; // 系数待测
        chassis_cmd_send.vx           = rc_data[TEMP].key[KEY_PRESS].a * 660 * 12 - rc_data[TEMP].key[KEY_PRESS].d * 660 * 12;
        chassis_cmd_send.wz           = rc_data[TEMP].key[KEY_PRESS].q * 660 * 12 - rc_data[TEMP].key[KEY_PRESS].e * 660 * 12;
    }

    else if (PC_Mode == PC_Get_Money) { // 兑矿模式0.
                                        //     // 前端
                                        //     if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].q) {
                                        //         // pitch归中
                                        //     } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a) {
                                        //         // roll归中
                                        //     } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].z || rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s) {
                                        //         // 一级伸出归中 （同样也是一级yaw归中
                                        //         first_stretch_cmd_send.first_stretch_mode = FIRST_INIT;
                                        //     } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) {
                                        //         // 二级伸出归中 （同样也是二级yaw归中，若禁用二级yaw无影响
                                        //         second_stretch_cmd_send.second_stretch_mode = SECOND_INIT;
                                        //     } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].e) {
                                        //         // 升降归位
                                        //         lift_cmd_send.lift_mode = LIFT_INIT;
                                        //     } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d) {
                                        //         // 横移归中



        // 这里左右电机默认镜像，若反转应改正负

        first_stretch_cmd_send.left_now += -rc_data[TEMP].key[KEY_PRESS].z * 66 + rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].z * 66 - rc_data[TEMP].key[KEY_PRESS].s * 66 + rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s * 66;
        first_stretch_cmd_send.right_now += -rc_data[TEMP].key[KEY_PRESS].z * 66 + rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].z * 66 + rc_data[TEMP].key[KEY_PRESS].s * 66 - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s * 66;

        second_stretch_cmd_send.left_now += rc_data[TEMP].key[KEY_PRESS].x * 20 - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x * 20;
        second_stretch_cmd_send.right_now -= rc_data[TEMP].key[KEY_PRESS].x * 20 - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x * 20;

        lift_cmd_send.left_now += rc_data[TEMP].key[KEY_PRESS].e * 28 - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e * 28;
        lift_cmd_send.right_now -= rc_data[TEMP].key[KEY_PRESS].e * 28 - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e * 28;

        horizontal_cmd_send.Now_MechAngle += rc_data[TEMP].key[KEY_PRESS].d * 11 - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d * 11;

        // servo_cmd_send.pitch_now_angle += rc_data[TEMP].rc.rocker_r1 / 660.0 * 10;
        // servo_cmd_send.yaw_now_angle += rc_data[TEMP].rc.rocker_l_ / 660.0 * 10;

        control_forward((rc_data[TEMP].key[KEY_PRESS].q * 120 - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q * 120), (rc_data[TEMP].key[KEY_PRESS].a * 120 - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].a * 120));
    }

    else if (PC_Mode == PC_To_Begin_ALL) {
        // 全回到初始位置
        // first_stretch_cmd_send.first_stretch_mode   = FIRST_INIT;
        // second_stretch_cmd_send.second_stretch_mode = SECOND_INIT;
        // lift_cmd_send.lift_mode                     = LIFT_INIT;
        // horizontal_cmd_send.Horizontal_mode         = HORIZONTAL_INIT;
    }

    else if (PC_Mode == DA_MIAO_Reset_All) {
        // 重新上电达妙板子
    }

    // gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    // gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;
}




/**
 * @brief 前端的控制逻辑
 *
 *
 */

int8_t mode; // pitch和roll的模式
int8_t last_mode;
float last_angle;      // pitch的最后一次编码器角度
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

    if ((!is_range(rc_data[TEMP].rc.rocker_l_) && is_range(rc_data[TEMP].rc.rocker_r1)) || (((rc_data[TEMP].key[KEY_PRESS].q) || (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q)) && (!(rc_data[TEMP].key[KEY_PRESS].a) || !(rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].a)))) {
        mode                          = PITCH_RUN_MODE;
        forward_cmd_send.Forward_mode = PITCH;
    } else if ((is_range(rc_data[TEMP].rc.rocker_l_) && !is_range(rc_data[TEMP].rc.rocker_r1)) || ((!(rc_data[TEMP].key[KEY_PRESS].q) || !(rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q)) && ((rc_data[TEMP].key[KEY_PRESS].a) || (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].a)))) {
        mode                          = ROLL_RUN_MODE;
        forward_cmd_send.Forward_mode = ROLL;
    } else {
        mode = STOP_MODE;
        forward_cmd_send.Forward_mode = ROLL;

    }
}





/**
 * @brief 自动模式控制
 *
 */

int32_t auto_decide_flag = 1, auto_confirm_flag = 0;
int16_t flag_r1, flag_r2, flag_r3, flag_r4;                                                  // 数据小心会溢出
void auto_mode_decide();                                                                     // 自动模式选择函数
void Put_it_back_in_the_silo();                                                              // 放回矿仓
void auto_small_resource_island();                                                           // 取小资源岛
float Automatic_mode_target_setting(float target, float measure, float expected_increments); // 自动模式目标值设定函数，防止一次给的值过于太大了，导致疯车

void auto_mode() // 自动模式最终函数
{

    if ((switch_is_up(rc_data[TEMP].rc.switch_right)) && switch_is_down(rc_data[TEMP].rc.switch_left)) // 右上左下
    {
        auto_mode_decide();
        if (rc_data[TEMP].rc.rocker_l_ > 200 || rc_data[TEMP].rc.rocker_l_ < -200) {
            auto_confirm_flag=1;
            auto_small_resource_island(); // 取小资源岛
            Put_it_back_in_the_silo();    // 扔矿仓
        } else {
            auto_confirm_flag=0;
            Maintain_current_posture(); // 维持当前姿态
        }
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
        auto_decide_flag = 1; // 左矿

    } else if (flag_r2 > 100) {
        auto_decide_flag = 2; // 中矿
    } else if (flag_r3 > 100) {
        auto_decide_flag = 3; // 右矿

    } else if (flag_r4 > 100) {
        auto_decide_flag = 4; // 收回矿仓
    }
}
void auto_small_resource_island() // 取小资源岛
{
    horizontal_cmd_send.Horizontal_mode = HORIZONTAL_MOVE;
    first_stretch_cmd_send.left_now     = Automatic_mode_target_setting(15416, first_stretch_fetch_data.new_left_encoder, 30);
    first_stretch_cmd_send.right_now    = Automatic_mode_target_setting(-15577, first_stretch_fetch_data.new_right_encoder, 30);
    lift_cmd_send.left_now              = Automatic_mode_target_setting(13229, lift_fetch_data.new_left_angle, 15);
    lift_cmd_send.right_now             = Automatic_mode_target_setting(-12907, lift_fetch_data.new_right_angle, 15);
    second_stretch_cmd_send.right_now   = Automatic_mode_target_setting(-25, second_stretch_fetch_data.new_right_angle, 15);
    second_stretch_cmd_send.left_now    = Automatic_mode_target_setting(65, second_stretch_fetch_data.new_left_angle, 15);
    switch (auto_decide_flag) {
        case 1: // 左矿
            if (second_stretch_fetch_data.new_left_angle > -5900 || second_stretch_fetch_data.new_right_angle < 6300) {
                horizontal_cmd_send.Now_MechAngle = Automatic_mode_target_setting(-13262, horizontal_fetch_data.Horizontal_Movement, 5);
            } else {
                horizontal_cmd_send.Now_MechAngle = horizontal_fetch_data.Horizontal_Movement;
            }
            break;
        case 2: // 中矿
            horizontal_cmd_send.Now_MechAngle = Automatic_mode_target_setting(27, horizontal_fetch_data.Horizontal_Movement, 5);
            break;
        case 3: // 右矿
            if (second_stretch_fetch_data.new_left_angle > -5900 || second_stretch_fetch_data.new_right_angle < 6300) {
                horizontal_cmd_send.Now_MechAngle = Automatic_mode_target_setting(11696, horizontal_fetch_data.Horizontal_Movement, 5);
            } else {
                horizontal_cmd_send.Now_MechAngle = horizontal_fetch_data.Horizontal_Movement;
            }
            break;
        default: // 以防autofalg出现问题
            horizontal_cmd_send.Now_MechAngle = horizontal_fetch_data.Horizontal_Movement;
            break;
    }
}
// 放回矿仓
void Put_it_back_in_the_silo()
{
    horizontal_cmd_send.Now_MechAngle = Automatic_mode_target_setting(27, horizontal_fetch_data.Horizontal_Movement, 5);
    if (horizontal_fetch_data.Horizontal_Movement < 1000 && horizontal_fetch_data.Horizontal_Movement > -1000) // 横移回到了二级的里面
    {
        second_stretch_cmd_send.left_now  = Automatic_mode_target_setting(-13678, second_stretch_fetch_data.new_left_angle, 15);
        second_stretch_cmd_send.right_now = Automatic_mode_target_setting(14096, second_stretch_fetch_data.new_right_angle, 15);
        first_stretch_cmd_send.left_now   = Automatic_mode_target_setting(23162, first_stretch_fetch_data.new_left_encoder, 30);
        first_stretch_cmd_send.right_now  = Automatic_mode_target_setting(-23849, first_stretch_fetch_data.new_right_encoder, 30);
        lift_cmd_send.left_now            = Automatic_mode_target_setting(7895, lift_fetch_data.new_left_angle, 15);
        lift_cmd_send.right_now           = Automatic_mode_target_setting(-8019, lift_fetch_data.new_right_angle, 15);
    } else {
        first_stretch_cmd_send.left_now   = first_stretch_fetch_data.new_left_encoder;
        first_stretch_cmd_send.left_now   = first_stretch_fetch_data.new_left_encoder;
        second_stretch_cmd_send.left_now  = second_stretch_fetch_data.new_left_angle;
        second_stretch_cmd_send.right_now = second_stretch_fetch_data.new_right_angle;
        lift_cmd_send.left_now            = lift_fetch_data.new_left_angle;
        lift_cmd_send.right_now           = lift_fetch_data.new_right_angle;
    }
}
// 保持当前姿态的函数，不止自动模式可以用，其他地方也可以
void Maintain_current_posture()
{
    first_stretch_cmd_send.left_now   = first_stretch_fetch_data.new_left_encoder;
    first_stretch_cmd_send.left_now   = first_stretch_fetch_data.new_left_encoder;
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
    if (measure - target < -20) {
        control_value = measure + expected_increments;
    } else if (measure - target > 20) {
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
    SubGetMessage(servo_feed_sub,(void *)&servo_fetch_data);
    // 遥控器其余状态为遥控器模式//遥控器左下右中，切换为电脑模式
    if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_mid(rc_data[TEMP].rc.switch_right)) {
        MouseKeySet();
    } else {
        RemoteControlSet();
    }
    // 遥控器左下右上   自动模式
    auto_mode();

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(lift_cmd_pub, (void *)&lift_cmd_send);
    PubPushMessage(first_stretch_cmd_pub, (void *)&first_stretch_cmd_send);
    PubPushMessage(second_stretch_cmd_pub, (void *)&second_stretch_cmd_send);
    PubPushMessage(forward_cmd_pub, (void *)&forward_cmd_send);
    PubPushMessage(horizontal_cmd_pub, (void *)&horizontal_cmd_send);
    PubPushMessage(servo_cmd_pub,(void *)&servo_cmd_send);
}
