#include "arm.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "encoder.h"
#include "DMmotor.h"
static DMMotorInstance *BIG_PITCH, *YAW, *BIG_ROLL, *SMALL_PITCH;
static DJIMotorInstance *SMALL_ROLL;

static Publisher_t *BIG_PITCH_pub , *YAW_pub , *BIG_ROLL_pub , *SMALL_PITCH_pub , *SMALL_ROLL_pub;
static Subscriber_t *BIG_PITCH_sub , *YAW_sub , *BIG_ROLL_sub , *SMALL_PITCH_sub , *SMALL_ROLL_sub; 

static BigPitch_Upload_Data_s BigPitch_feedback_data; 
static BigPitch_Ctrl_Cmd_s BigPitch_cmd_recv;       

static YAW_Upload_Data_s YAW_feedback_data; 
static YAW_Ctrl_Cmd_s YAW_cmd_recv;       

static BIG_ROLL_Upload_Data_s BIG_ROLL_feedback_data;
static BIG_ROLL_Ctrl_Cmd_s BIG_ROLL_cmd_recv; 

static SMALL_PITCH_Upload_Data_s SMALL_PITCH_feedback_data;
static SMALL_PITCH_Ctrl_Cmd_s SMALL_PITCH_cmd_recv; 

static SMALL_ROLL_Upload_Data_s SMALL_ROLL_feedback_data;
static SMALL_ROLL_Ctrl_Cmd_s SMALL_ROLL_cmd_recv; 


void ARM_Init()
{
    Motor_Init_Config_s BIG_PITCH_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
        },
        .motor_type = DM8006,
        
    };
    BIG_PITCH_config.can_init_config.rx_id = 0x05;
    BIG_PITCH_config.can_init_config.tx_id = 0x105;
    BIG_PITCH = DMMotorInit(&BIG_PITCH_config);


    BIG_PITCH_pub = PubRegister("BIG_PITCH_feed", sizeof(BigPitch_Upload_Data_s));
    BIG_PITCH_sub = SubRegister("BIG_PITCH_cmd", sizeof(BigPitch_Ctrl_Cmd_s));

    Motor_Init_Config_s YAW_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .motor_type = DM6006,

    };
    YAW_config.can_init_config.rx_id = 0x003;
    YAW_config.can_init_config.tx_id = 0x103;
    YAW = DMMotorInit(&YAW_config);

    YAW_pub = PubRegister("YAW_feed", sizeof(YAW_Upload_Data_s));
    YAW_sub = SubRegister("YAW_cmd", sizeof(YAW_Ctrl_Cmd_s));

    Motor_Init_Config_s SMALL_PITCH_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .motor_type = DM4310,
        
    };
    SMALL_PITCH_config.can_init_config.rx_id = 0x2;
    SMALL_PITCH_config.can_init_config.tx_id = 0x102;
    SMALL_PITCH = DMMotorInit(&SMALL_PITCH_config);

    SMALL_PITCH_pub = PubRegister("SMALL_PITCH_feed", sizeof(SMALL_PITCH_Upload_Data_s));
    SMALL_PITCH_sub = SubRegister("SMALL_PITCH_cmd", sizeof(SMALL_PITCH_Ctrl_Cmd_s));

     

    

    Motor_Init_Config_s BIG_ROLL_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
        },
        .motor_type = DM4310,

    };
    BIG_ROLL_config.can_init_config.rx_id = 0x1;
    BIG_ROLL_config.can_init_config.tx_id = 0x101;
    BIG_ROLL = DMMotorInit(&BIG_ROLL_config);

    BIG_ROLL_pub = PubRegister("BIG_ROLL_feed", sizeof(BIG_ROLL_Upload_Data_s));
    BIG_ROLL_sub = SubRegister("BIG_ROLL_cmd", sizeof(BIG_ROLL_Ctrl_Cmd_s));




     Motor_Init_Config_s SMALL_ROLL_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1.2, // 4.5
                .Ki            = 0.1,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 200,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006};
    SMALL_ROLL_config.can_init_config.tx_id    = 2;

    SMALL_ROLL = DJIMotorInit(&SMALL_ROLL_config);
    SMALL_ROLL_pub = PubRegister("SMALL_ROLL_feed", sizeof(SMALL_ROLL_Upload_Data_s));
    SMALL_ROLL_sub = SubRegister("SMALL_ROLL_cmd", sizeof(SMALL_ROLL_Ctrl_Cmd_s));
    DMMotorStop(BIG_PITCH);
    DMMotorStop(BIG_ROLL);
    DMMotorStop(YAW);
    DMMotorStop(SMALL_PITCH);
    DJIMotorStop(SMALL_ROLL);
}
void ARM_Task()
{
    SubGetMessage(BIG_PITCH_sub, &BigPitch_cmd_recv);
    SubGetMessage(YAW_sub, &YAW_cmd_recv);
    SubGetMessage(BIG_ROLL_sub, &BIG_ROLL_cmd_recv);
    SubGetMessage(SMALL_PITCH_sub, &SMALL_PITCH_cmd_recv);
    SubGetMessage(SMALL_ROLL_sub, &SMALL_ROLL_cmd_recv);
    
    if(BigPitch_cmd_recv.BIG_PITCH_mode == BIG_PITCH_START)
    {
        DMMotorEnable(BIG_PITCH);
        DMMotorEnable(BIG_ROLL);
        DMMotorEnable(YAW);
        DMMotorEnable(SMALL_PITCH);
        DJIMotorEnable(SMALL_ROLL);
    }
    else if (BigPitch_cmd_recv.BIG_PITCH_mode == BIG_PITCH_STOP)
    {
        DMMotorStop(BIG_PITCH);
        DMMotorStop(BIG_ROLL);
        DMMotorStop(YAW);
        DMMotorStop(SMALL_PITCH);
        DJIMotorStop(SMALL_ROLL);
    }

    DMMotorSetRef(BIG_ROLL,6,BIG_ROLL_cmd_recv.angle);
    DMMotorSetRef(BIG_PITCH,10,BigPitch_cmd_recv.angle); 
    DMMotorSetRef(YAW,2,YAW_cmd_recv.angle);
    DMMotorSetRef(SMALL_PITCH,5,SMALL_PITCH_cmd_recv.angle);

    DJIMotorSetRef(SMALL_ROLL,SMALL_ROLL_cmd_recv.speed);


    BigPitch_feedback_data.now_angle = BIG_PITCH->measure.pos;
    YAW_feedback_data.now_angle = YAW->measure.pos;
    BIG_ROLL_feedback_data.now_angle = BIG_ROLL->measure.pos;
    SMALL_PITCH_feedback_data.now_angle = SMALL_PITCH->measure.pos;
    SMALL_ROLL_feedback_data.now_angle = SMALL_ROLL->measure.speed_aps;

    BigPitch_feedback_data.torque = BIG_PITCH->measure.tor;
    YAW_feedback_data.torque = YAW->measure.tor;
    BIG_ROLL_feedback_data.torque = BIG_ROLL->measure.tor;
    SMALL_PITCH_feedback_data.torque = SMALL_PITCH->measure.tor;

    BigPitch_feedback_data.speed = BIG_PITCH->measure.vel;
    YAW_feedback_data.speed = YAW->measure.vel;
    BIG_ROLL_feedback_data.speed = BIG_ROLL->measure.vel;
    SMALL_PITCH_feedback_data.speed = SMALL_PITCH->measure.vel;

    PubPushMessage(BIG_PITCH_pub, (void *)&BigPitch_feedback_data);
    PubPushMessage(YAW_pub, (void *)&YAW_feedback_data);
    PubPushMessage(BIG_ROLL_pub, (void *)&BIG_ROLL_feedback_data);
    PubPushMessage(SMALL_PITCH_pub, (void *)&SMALL_PITCH_feedback_data);
    PubPushMessage(SMALL_ROLL_pub, (void *)&SMALL_ROLL_feedback_data);


}