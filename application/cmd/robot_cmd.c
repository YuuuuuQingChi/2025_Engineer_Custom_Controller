// app
#include "robot_def.h"
#include "robot_cmd.h"
//#include "second.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "buzzer.h"
//#include "forward.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

//以后这块搞点宏
#define PITCH_RUN_MODE 2
#define ROLL_RUN_MODE 3
#define STOP_MODE 4
#define Rotation_Ratio 1.5

//111

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回

static Publisher_t *first_stretch_cmd_pub;            // 一级控制消息发布者
static Subscriber_t *first_stretch_feed_sub;          // 一级反馈信息订阅者
static First_Stretch_Ctrl_Cmd_s first_stretch_cmd_send;      // 传递给一级的控制信息
static First_Stretch_Upload_Data_s first_stretch_fetch_data; // 从一级获取的反馈信息

static Publisher_t *second_stretch_cmd_pub;            // 二级控制消息发布者
static Subscriber_t *second_stretch_feed_sub;          // 二级反馈信息订阅者
static Second_Stretch_Ctrl_Cmd_s second_stretch_cmd_send;      // 传递给二级的控制信息
static Second_Stretch_Upload_Data_s second_stretch_fetch_data; // 从二级获取的反馈信息

static Publisher_t *lift_cmd_pub;            // 升降控制消息发布者
static Subscriber_t *lift_feed_sub;          // 升降反馈信息订阅者
static Lift_Ctrl_Cmd_s lift_cmd_send;      // 传递给升降的控制信息
static Lift_Upload_Data_s lift_fetch_data; // 从升降获取的反馈信息

static Publisher_t *horizontal_cmd_pub;            // 横移控制消息发布者
static Subscriber_t *horizontal_feed_sub;          // 横移反馈信息订阅者  
static Horizontal_Ctrl_Cmd_s horizontal_cmd_send;      // 传递给横移的控制信息
static Horizontal_Upload_Data_s horizontal_fetch_data; // 从横移获取的反馈信息

static Publisher_t *forward_cmd_pub;            // 前端控制消息发布者
static Subscriber_t *forward_feed_sub;          // 前端反馈信息订阅者
static Forward_Ctrl_Cmd_s forward_cmd_send;      // 传递给前端的控制信息
static Forward_Upload_Data_s forward_fetch_data; // 从前端获取的反馈信息

static Publisher_t *servo_cmd_pub;            // 升降控制消息发布者
static Subscriber_t *servo_feed_sub;          // 升降反馈信息订阅者
static Servo_Cmd_s servo_cmd_send;      // 传递给升降的控制信息
static Servo_Upload_Data_s servo_fetch_data; // 从升降获取的反馈信息

PC_Mode_t PC_Mode;
extern PIDInstance *encoder_pid;
static Robot_Status_e robot_state; // 机器人整体工作状态

//前端
int8_t mode;           // pitch和roll的模式
int8_t last_mode;
float last_angle;     // pitch的最后一次编码器角度
float relevant_angle; // pitch和roll的相对角度
float final_angle;
// 动之前的roll编码器
float roll_last_angle; // roll的最后一次编码器角度

void RobotCMDInit()
{
    rc_data          = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    
    lift_cmd_pub = PubRegister("lift_cmd", sizeof(Lift_Ctrl_Cmd_s));
    lift_feed_sub = SubRegister("lift_feed", sizeof(Lift_Upload_Data_s));
    first_stretch_cmd_pub  = PubRegister("first_stretch_cmd", sizeof(First_Stretch_Ctrl_Cmd_s));  
     first_stretch_feed_sub = SubRegister("first_stretch_feed", sizeof(First_Stretch_Upload_Data_s));
    second_stretch_cmd_pub  = PubRegister("second_stretch_cmd", sizeof(Second_Stretch_Ctrl_Cmd_s));
     second_stretch_feed_sub = SubRegister("second_stretch_feed", sizeof(Second_Stretch_Upload_Data_s));
     chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
     chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    forward_cmd_pub  = PubRegister("forward_cmd", sizeof(Forward_Ctrl_Cmd_s));
     forward_feed_sub = SubRegister("forward_feed", sizeof(Forward_Upload_Data_s));
    horizontal_cmd_pub = PubRegister("Horizontal_cmd", sizeof(Horizontal_Ctrl_Cmd_s));
    horizontal_feed_sub = SubRegister("Horizontal_feed", sizeof(Horizontal_Upload_Data_s));

    servo_cmd_pub = PubRegister("servo_cmd", sizeof(Servo_Cmd_s));
    servo_feed_sub = SubRegister("servo_feed", sizeof(Servo_Upload_Data_s));

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
    // chassis_cmd_send.chassis_mode=CHASSIS_WALK;
    // horizontal_cmd_send.Horizontal_mode=HORIZONTAL_MOVE;
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


}
void Init_Value(){
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

void mode_record();
void control_forward();

float last_first_right_angle,last_first_left_angle;
float last_second_right_angle,last_second_left_angle;
float last_lift_right_angle,last_lift_left_angle;
float last_horizontal_angle;

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */

int is_range(int a){
    if ((a> 2)||(a< -2)){
        return 1;
    }
    else {
        return 0;
    }
}
static void RemoteControlSet()
{
    if ((switch_is_up(rc_data[TEMP].rc.switch_right))&&switch_is_up(rc_data[TEMP].rc.switch_left)) {
        if(is_range(rc_data[TEMP].rc.rocker_l1)||is_range(rc_data[TEMP].rc.rocker_l_)||is_range(rc_data[TEMP].rc.rocker_r_)){
            chassis_cmd_send.chassis_mode=CHASSIS_WALK;
            chassis_cmd_send.vx = -rc_data[TEMP].rc.rocker_l_*20; // 系数待测
            chassis_cmd_send.vy = -rc_data[TEMP].rc.rocker_l1*20;
            chassis_cmd_send.wz =-rc_data[TEMP].rc.rocker_r_*15;
        }


        if(is_range(rc_data[TEMP].rc.rocker_r1))
        {
            lift_cmd_send.lift_mode = LIFT;
            lift_cmd_send.left_now += rc_data[TEMP].rc.rocker_r1/20.0;
            lift_cmd_send.right_now -= rc_data[TEMP].rc.rocker_r1/20.0;
            lift_cmd_send.left_last=lift_cmd_send.left_now;
            lift_cmd_send.right_last=lift_cmd_send.right_now;
        }
        
    }

    // 右侧开关状态[中],左侧开关状态[中]
    if ((switch_is_mid(rc_data[TEMP].rc.switch_right))&&switch_is_mid(rc_data[TEMP].rc.switch_left)) 
    {
        //一级伸出 yaw 
        if(1-is_range(rc_data[TEMP].rc.rocker_l_)&&(is_range(rc_data[TEMP].rc.rocker_r1)))
        {
            first_stretch_cmd_send.first_stretch_mode = FIRST_STRETCH;
            first_stretch_cmd_send.left_now -= rc_data[TEMP].rc.rocker_r1/10.0;
            first_stretch_cmd_send.right_now+=rc_data[TEMP].rc.rocker_r1/10.0;
            first_stretch_cmd_send.right_last=first_stretch_cmd_send.right_now;
            first_stretch_cmd_send.left_last=first_stretch_cmd_send.left_now;
        }
        else
        {
            first_stretch_cmd_send.right_now=first_stretch_cmd_send.right_last;
            first_stretch_cmd_send.left_now=first_stretch_cmd_send.left_last;
        }
    
        //二级伸出 
        if(is_range(rc_data[TEMP].rc.rocker_l1))
        {
            second_stretch_cmd_send.second_stretch_mode = SECOND_STRETCH;
            second_stretch_cmd_send.left_now += rc_data[TEMP].rc.rocker_l1/23.0;
            second_stretch_cmd_send.right_now -= rc_data[TEMP].rc.rocker_l1/23.0;
            second_stretch_cmd_send.right_last=second_stretch_cmd_send.right_now;
            second_stretch_cmd_send.left_last=second_stretch_cmd_send.left_now;
        }
        
        else if(1-is_range(rc_data[TEMP].rc.rocker_l1))
        {
            second_stretch_cmd_send.right_now=second_stretch_cmd_send.right_last;
            second_stretch_cmd_send.left_now=second_stretch_cmd_send.left_last;
        }
    
    } 

    // 右侧开关状态[上],左侧开关状态[中]
    if ((switch_is_up(rc_data[TEMP].rc.switch_right))&&switch_is_mid(rc_data[TEMP].rc.switch_left)) 
    {
        //横移
        if(is_range(rc_data[TEMP].rc.rocker_r_))
        {
            horizontal_cmd_send.Now_MechAngle += rc_data[TEMP].rc.rocker_r_/60.0;
            horizontal_cmd_send.Last_MechAngle = horizontal_cmd_send.Now_MechAngle;
        }
        else if(1 - is_range(rc_data[TEMP].rc.rocker_r_))
        {
            horizontal_cmd_send.Now_MechAngle =  horizontal_cmd_send.Last_MechAngle ;
        }

         if (1-is_range(rc_data[TEMP].rc.rocker_r1)&&(is_range(rc_data[TEMP].rc.rocker_l_)))
        {
            first_stretch_cmd_send.first_stretch_mode = FIRST_YAW;
            first_stretch_cmd_send.left_now -= rc_data[TEMP].rc.rocker_l_/10.0;
            first_stretch_cmd_send.right_now-= rc_data[TEMP].rc.rocker_l_/10.0;
            first_stretch_cmd_send.right_last=first_stretch_cmd_send.right_now;
            first_stretch_cmd_send.left_last=first_stretch_cmd_send.left_now;
        }
        else
        {
            first_stretch_cmd_send.right_now=first_stretch_cmd_send.right_last;
            first_stretch_cmd_send.left_now=first_stretch_cmd_send.left_last;
        }

        if (is_range(rc_data[TEMP].rc.dial)){
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,1);
            
        }else{
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,0 );

        }
        //前端
        // control_forward();
        // mode_record();

    }

    //双下
    if ((switch_is_down(rc_data[TEMP].rc.switch_right))&&switch_is_down(rc_data[TEMP].rc.switch_left)){
        // lift_cmd_send.left_now=0;
        // lift_cmd_send.right_now=0;
        // lift_cmd_send.left_last=0;
        // lift_cmd_send.right_last=0;

        // first_stretch_cmd_send.left_now=0;
        // first_stretch_cmd_send.right_now=0;
        // first_stretch_cmd_send.left_last=0;
        // first_stretch_cmd_send.right_last=0;

        // second_stretch_cmd_send.left_now=STRETCH_2_INIT_ANGLE_LEFT;
        // second_stretch_cmd_send.right_now=STRETCH_2_INIT_ANGLE_RIGHT;
        // second_stretch_cmd_send.left_last=STRETCH_2_INIT_ANGLE_LEFT;
        // second_stretch_cmd_send.right_last=STRETCH_2_INIT_ANGLE_RIGHT;
        //lift_cmd_send.lift_mode = LIFT_STOP;
        chassis_cmd_send.chassis_mode=CHASSIS_ZERO_FORCE;
        if (is_range(rc_data[TEMP].rc.dial))
        {
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
            //重新上电
        
    }

    
    //差个升降的限位，上车测数据再说
    //以上是升降的控制逻辑
    lift_cmd_send.left_now=lift_cmd_send.left_last;
    lift_cmd_send.right_now=lift_cmd_send.right_last;
    lift_cmd_send.left_now=Limit_Set(lift_cmd_send.left_now,LIFT_MAX_ANGLE_LEFT,LIFT_MIN_ANGLE_LEFT);    
    lift_cmd_send.right_now=Limit_Set(lift_cmd_send.right_now,LIFT_MAX_ANGLE_RIGHT,LIFT_MIN_ANGLE_RIGHT);
    lift_cmd_send.left_last=Limit_Set(lift_cmd_send.left_now,LIFT_MAX_ANGLE_LEFT,LIFT_MIN_ANGLE_LEFT);    
    lift_cmd_send.right_last=Limit_Set(lift_cmd_send.right_now,LIFT_MAX_ANGLE_RIGHT,LIFT_MIN_ANGLE_RIGHT);

    first_stretch_cmd_send.left_now=Limit_Set(first_stretch_cmd_send.left_now,24000,-38000);    
    first_stretch_cmd_send.right_now=Limit_Set(first_stretch_cmd_send.right_now,STRETCH_1_MAX_ANGLE_RIGHT,STRETCH_1_MIN_ANGLE_RIGHT);
    first_stretch_cmd_send.left_last=Limit_Set(first_stretch_cmd_send.left_now,24000,-38000);    
    first_stretch_cmd_send.right_last=Limit_Set(first_stretch_cmd_send.right_now,STRETCH_1_MAX_ANGLE_RIGHT,STRETCH_1_MIN_ANGLE_RIGHT);

    second_stretch_cmd_send.left_now=Limit_Set(second_stretch_cmd_send.left_now,STRETCH_2_MAX_ANGLE_LEFT,STRETCH_2_MIN_ANGLE_LEFT);    
    second_stretch_cmd_send.right_now=Limit_Set(second_stretch_cmd_send.right_now,STRETCH_2_MAX_ANGLE_RIGHT,STRETCH_2_MIN_ANGLE_RIGHT);
    second_stretch_cmd_send.left_last=Limit_Set(second_stretch_cmd_send.left_now,STRETCH_2_MAX_ANGLE_LEFT,STRETCH_2_MIN_ANGLE_LEFT);    
    second_stretch_cmd_send.right_last=Limit_Set(second_stretch_cmd_send.right_now,STRETCH_2_MAX_ANGLE_RIGHT,STRETCH_2_MIN_ANGLE_RIGHT);

    horizontal_cmd_send.Now_MechAngle=Limit_Set(horizontal_cmd_send.Now_MechAngle,HORIZONTAL_MAX,HORIZONTAL_MIN);

    //气泵
        if (is_range(rc_data[TEMP].rc.dial)){
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,1);
            
        }else{
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,0 );

        }
}

float Limit_Set(float obj,int max,int min){
    if (obj>max){
        obj=max;
    }
    if (obj<min){
        obj=min;
    }
    return obj;
}
/** @todo 每个部位的复位
 * @brief 键盘模式控制
 *
 */

void PC_Mode_Set(PC_Mode_t * mode){
    if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].w){
        *mode=PC_Walk;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].s){
        *mode=PC_Get_Money;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].f){
        *mode=PC_To_Begin_ALL;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT_AND_CTRL].r){
        *mode=DA_MIAO_Reset_All;
    }
    // 以上是四种大模式的判断

}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{   
    PC_Mode_Set(&PC_Mode);

    if (PC_Mode==PC_Walk){
        //行走模式
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;
    chassis_cmd_send.wz = rc_data[TEMP].key[KEY_PRESS].q * 300 - rc_data[TEMP].key[KEY_PRESS].e * 300;
    }

    else if (PC_Mode==PC_Get_Money){//兑矿模式0.
        //前端
        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].q){
            //pitch归中
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a){
            //roll归中
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].z ||rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s){
            //一级伸出归中 （同样也是一级yaw归中
            first_stretch_cmd_send.first_stretch_mode=FIRST_INIT;
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x){
            //二级伸出归中 （同样也是二级yaw归中，若禁用二级yaw无影响
            second_stretch_cmd_send.second_stretch_mode=SECOND_INIT;
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].e){
            //升降归位
            lift_cmd_send.lift_mode=LIFT_INIT;
        }
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d){
            //横移归中

            // @todo 
            //为了防止干涉，这里应该判断伸出状态，再决定是否归位
            //等转正了再完善（bushi）
            horizontal_cmd_send.Horizontal_mode=HORIZONTAL_INIT;

        }

        //这里左右电机默认镜像，若反转应改正负
        first_stretch_cmd_send.left_now+=(rc_data[TEMP].key[KEY_PRESS].z*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].z*100+rc_data[TEMP].key[KEY_PRESS].s*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s*100);
        first_stretch_cmd_send.right_now+=(rc_data[TEMP].key[KEY_PRESS].z*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].z*100-rc_data[TEMP].key[KEY_PRESS].s*100+rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s*100);

        second_stretch_cmd_send.left_now+=rc_data[TEMP].key[KEY_PRESS].x*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x*100;
        second_stretch_cmd_send.left_now-=rc_data[TEMP].key[KEY_PRESS].x*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x*100;
        
        lift_cmd_send.left_now+=rc_data[TEMP].key[KEY_PRESS].x*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x*100;
        lift_cmd_send.right_now-=rc_data[TEMP].key[KEY_PRESS].x*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].x*100;

        horizontal_cmd_send.Now_MechAngle+=rc_data[TEMP].key[KEY_PRESS].d*100-rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d*100;
    }

    else if (PC_Mode==PC_To_Begin_ALL){
        //全回到初始位置
        first_stretch_cmd_send.first_stretch_mode=FIRST_INIT;
        second_stretch_cmd_send.second_stretch_mode=SECOND_INIT;
        lift_cmd_send.lift_mode=LIFT_INIT;
        horizontal_cmd_send.Horizontal_mode=HORIZONTAL_INIT;
    }

    else if (PC_Mode==DA_MIAO_Reset_All){
        //重新上电达妙板子
    }


    // gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    // gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;
}
void mode_change();
void control_forward()
{
    last_mode = mode;
    mode_change();
    if (mode ==PITCH_RUN_MODE) //pitch
    {
        last_angle = forward_fetch_data.new_left_angle;
    }
    if (mode == ROLL_RUN_MODE) //roll
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
    final_angle  = rc_data[TEMP].rc.rocker_l_/660.0*30 - rc_data[TEMP].rc.rocker_r1/660.0*30 + last_angle - relevant_angle; 
    forward_cmd_send.angel_output = PIDCalculate(encoder_pid, forward_fetch_data.new_left_angle,final_angle);

    if (mode == ROLL_RUN_MODE) {
        forward_cmd_send.angel_output1 = -forward_cmd_send.angel_output;
    } else {
        forward_cmd_send.angel_output1 = forward_cmd_send.angel_output;
    }

}

void mode_change()
{
    
        if (!is_range(rc_data[TEMP].rc.rocker_l_)&&is_range(rc_data[TEMP].rc.rocker_r1)) {
            mode = PITCH_RUN_MODE;
            forward_cmd_send.Forward_mode = PITCH;
        } else if (is_range(rc_data[TEMP].rc.rocker_l_)&&!is_range(rc_data[TEMP].rc.rocker_r1)) {
            mode = ROLL_RUN_MODE;
            forward_cmd_send.Forward_mode = ROLL;
        } 
        else
        {
            mode = ROLL_RUN_MODE;
            mode = 5;
        }
}




int16_t auto_decide_flag = 1,auto_confirm_flag = 0;
int flag_r1,flag_r2,flag_r3,flag_r4;//数据小心会溢出
int confirm_flag;

void auto_mode_decide()
{
    
    
    if((switch_is_up(rc_data[TEMP].rc.switch_right))&&switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        if (rc_data[TEMP].rc.rocker_r_ >= 200 && (!is_range(rc_data[TEMP].rc.rocker_l_)))
        {
        flag_r1++;
        }
        else if(!is_range(rc_data[TEMP].rc.rocker_r_))
        {
        flag_r1 = 0;
        }
        if (rc_data[TEMP].rc.rocker_r_ <= -200 && (!is_range(rc_data[TEMP].rc.rocker_l_)))
        {
        flag_r2++;
        }
        else if(!is_range(rc_data[TEMP].rc.rocker_r_))
        {
        flag_r2 = 0;
        }
        if (rc_data[TEMP].rc.rocker_r1 >= 200 && (!is_range(rc_data[TEMP].rc.rocker_l_)))
        {
        flag_r3++;
        }
        else if(!is_range(rc_data[TEMP].rc.rocker_r1))
        {
        flag_r3 = 0;
        }
        if(flag_r1 > 200)
        {
            auto_decide_flag = 1;//左矿
            
        }
        else if(flag_r2 > 200)
        {
            auto_decide_flag = 2;//中矿
        }
        else if(flag_r3 > 200)
        {
            auto_decide_flag = 3;//右矿
            
        }
        else if(flag_r4 > 200)
        {
            auto_decide_flag = 4;
            
        }
       
    }
    
}
// void auto_mode_confirm()
// {
//     if((rc_data[TEMP].rc.rocker_l_ > 2 || rc_data[TEMP].rc.rocker_l_ < -2) && !is_range(rc_data[LAST].rc.rocker_l_))
//     {
//         auto_confirm_flag = 0;
//     }
//     else if (((rc_data[TEMP].rc.rocker_r_ > 2 || rc_data[TEMP].rc.rocker_r_ < -2)) && ((rc_data[LAST].rc.rocker_r_ > 2|| rc_data[LAST].rc.rocker_r_ < -2)))
//     {
//         auto_confirm_flag = 1;
//     }
// }
//数值是0就是没测呢
void auto_mode()
{
    auto_mode_decide();
    //auto_mode_confirm();
    if((switch_is_up(rc_data[TEMP].rc.switch_right))&&switch_is_down(rc_data[TEMP].rc.switch_left))//右上左下
    {
        //取矿
        if(rc_data[TEMP].rc.rocker_l_ > 2 || rc_data[TEMP].rc.rocker_l_ < -2)
        {
            first_stretch_cmd_send.left_now = 15416;
            first_stretch_cmd_send.right_now = -15577;
            lift_cmd_send.left_now =13229;
            lift_cmd_send.right_now =-12907;
            second_stretch_cmd_send.right_now =-25;
            second_stretch_cmd_send.left_now = 65;
            switch (auto_decide_flag)
            {
            case 1://左矿
            if(second_stretch_fetch_data.new_left_angle > -5900 || second_stretch_fetch_data.new_right_angle < 6300)
            {
                horizontal_cmd_send.Now_MechAngle = -13262;
            }
            else
            {
                horizontal_cmd_send.Now_MechAngle  = horizontal_fetch_data.Horizontal_Movement;
            }
            
                break;
            case 2://中矿
                horizontal_cmd_send.Now_MechAngle = 27;
                break;
            case 3://右矿
           if(second_stretch_fetch_data.new_left_angle > -5900 || second_stretch_fetch_data.new_right_angle < 6300)
            {
                horizontal_cmd_send.Now_MechAngle = 11696;
            }
            else
            {
                horizontal_cmd_send.Now_MechAngle  = horizontal_fetch_data.Horizontal_Movement;
            }
                break;
            default://以防autofalg出现问题
                horizontal_cmd_send.Now_MechAngle = horizontal_fetch_data.Horizontal_Movement;
            break;

            } 
     
            
        }
    }
    //扔肚子
    // else if(((switch_is_up(rc_data[LAST].rc.switch_right))&&switch_is_down(rc_data[LAST].rc.switch_left))&&((switch_is_mid(rc_data[TEMP].rc.switch_right))&&switch_is_down(rc_data[TEMP].rc.switch_left)))//从右上左下变成右上左中
    // {
        
    //     lift_cmd_send.left_now =0;
    //     lift_cmd_send.right_now =0;
    //     while(1)
    //     {
    //         horizontal_cmd_send.Now_MechAngle = 0;
    //         if(horizontal_fetch_data.Horizontal_Movement < 0 ||horizontal_fetch_data.Horizontal_Movement > 0)//横移回到了二级的里面
    //         {
    //             second_stretch_cmd_send.left_now = 0;
    //             second_stretch_cmd_send.right_now = 0;
    //         }
    //         if(second_stretch_fetch_data.new_left_angle<0||second_stretch_fetch_data.new_right_angle>0)//二级达到位置
    //         {
    //             break;
    //         }
    //     }
    //     first_stretch_cmd_send.left_now = 0;
    //     first_stretch_cmd_send.right_now = 0;

    // }

}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
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
       //遥控器左下右中，切换为电脑模式
    //遥控器其余状态为遥控器模式
//     if (switch_is_down(rc_data[TEMP].rc.switch_left)&&switch_is_mid(rc_data[TEMP].rc.switch_right)){
//         MouseKeySet();
//    }
//    else {
    RemoteControlSet();
    auto_mode();
   //}
   PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
   PubPushMessage(lift_cmd_pub, (void *)&lift_cmd_send);
   PubPushMessage(first_stretch_cmd_pub, (void *)&first_stretch_cmd_send);
   PubPushMessage(second_stretch_cmd_pub, (void *)&second_stretch_cmd_send);
   PubPushMessage(forward_cmd_pub, (void *)&forward_cmd_send);
   PubPushMessage(horizontal_cmd_pub, (void *)&horizontal_cmd_send);
   PubPushMessage(servo_cmd_pub, (void *)&servo_cmd_send);
}
