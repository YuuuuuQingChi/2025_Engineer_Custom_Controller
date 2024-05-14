#include "ui.h"
#include "referee_UI.h"
#include "message_center.h"
#include "cmsis_os.h"
#include "robot_cmd.h"
#include "robot.h"
#include "robot_cmd.h"
#include "remote_control.h"

static Publisher_t *ui_pub;                    // 用于发布底盘的数据
static Subscriber_t *ui_sub;                   // 用于订阅底盘的控制命令                                        // !ONE_BOARD
static ui_Cmd_s ui_cmd_recv;         // 底盘接收到的控制命令
static ui_Upload_Data_s ui_feedback_data; // 底盘回传的反馈数据


extern int32_t auto_decide_flag, auto_confirm_flag;
extern int16_t flag_r1, flag_r2, flag_r3, flag_r4;                                                  // 数据小心会溢出
extern referee_info_t referee_info;                         // 裁判系统数据
extern Referee_Interactive_info_t Referee_Interactive_info; // 绘制UI所需的数据
extern int32_t flag_referee_init;
uint8_t UI_Seq;                           // 包序号，供整个referee文件使用
static Graph_Data_t UI_Deriction_line[4]; // 射击准线
Graph_Data_t UI_Rectangle[10];     // 矩形
static Graph_Data_t UI_Circle_t[10];      // 圆形
static String_Data_t UI_State_sta[10];
static Graph_Data_t UI_Number_t[10];      // 数字

static Publisher_t *ui_pub;                    // 用于发布底盘的数据
static Subscriber_t *ui_sub;                   // 用于订阅底盘的控制命令

extern PC_Mode_t PC_Mode;

void uiInit()
{
    UI_Circle_t[0].start_x=120;
    UI_Circle_t[1].start_x=600;
    UI_Circle_t[2].start_x=680;
    UI_Circle_t[3].start_x=760;
    UI_Circle_t[4].start_x=840;

    UI_Circle_t[0].start_y=650;
    UI_Circle_t[1].start_y=130;
    UI_Circle_t[2].start_y=130;
    UI_Circle_t[3].start_y=130;
    UI_Circle_t[4].start_y=130;
    

    ui_sub = SubRegister("ui_cmd", sizeof(ui_Cmd_s));
    ui_pub = PubRegister("ui_feed", sizeof(ui_Upload_Data_s));



}
void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_info.referee_id.Robot_Color       = referee_info.GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_info.referee_id.Cilent_ID         = 0x0100 + referee_info.GameRobotState.robot_id; // 计算客户端ID
    referee_info.referee_id.Robot_ID          = referee_info.GameRobotState.robot_id;          // 计算机器人ID
    referee_info.referee_id.Receiver_Robot_ID = 0;
}


void uiTask()
{
    SubGetMessage(ui_sub, &ui_cmd_recv);
    PubPushMessage(ui_pub, (void *)&ui_feedback_data);
}

void My_UIGraphRefresh()
{

    DeterminRobotID();


    if (ui_cmd_recv.flag_refresh_ui){
        ui_feedback_data.flag_refresh_ui=0;
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 9);
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 8);
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 7);
	
    //UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_ADD, 9, UI_Color_White, 10, UI_Circle_t[0].start_x,UI_Circle_t[0].start_y, 8);
    UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_ADD, 9, UI_Color_White, 10, UI_Circle_t[1].start_x,UI_Circle_t[1].start_y, 15);
    UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_ADD, 9, UI_Color_White, 10, UI_Circle_t[2].start_x,UI_Circle_t[2].start_y, 15);
    UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_ADD, 9, UI_Color_White, 10, UI_Circle_t[3].start_x,UI_Circle_t[3].start_y, 15);
    UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_ADD, 9, UI_Color_White, 10, UI_Circle_t[4].start_x,UI_Circle_t[4].start_y, 15);
    //  UICircleDraw(&UI_Circle_t[5], "sc4", UI_Graph_ADD, 9, UI_Color_White, 10, 550 , 130, 8);
    UILineDraw(&UI_Deriction_line[0],"sl0",UI_Graph_ADD,9,UI_Color_Green,2,600,130,700,400);
    UILineDraw(&UI_Deriction_line[1],"sl1",UI_Graph_ADD,9,UI_Color_Green,2,1920-600,130,1220,400);

    UIRectangleDraw(&UI_Rectangle[0],"sj0",UI_Graph_ADD,9,UI_Color_Green,3,700,400,1220,700);

    sprintf(UI_State_sta[0].show_Data, "Sucker");
    UICharDraw(&UI_State_sta[0], "ss4", UI_Graph_ADD, 9, UI_Color_Yellow, 12, 2, 100, 700, "Sucker");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[0]);

    UIIntDraw(&UI_Number_t[0],"sm0",UI_Graph_ADD,9,UI_Color_White,20,3,300,700,ui_cmd_recv.auto_confirm_flag);
    UIIntDraw(&UI_Number_t[1],"sm1",UI_Graph_ADD,9,UI_Color_White,20,3,300,700,ui_cmd_recv.auto_decide_flag);
    UIGraphRefresh(&referee_info.referee_id, 5, UI_Deriction_line[0], UI_Deriction_line[1], UI_Deriction_line[2], UI_Deriction_line[3], UI_Circle_t[0]);
    UIGraphRefresh(&referee_info.referee_id, 2, UI_Number_t[0],UI_Number_t[1]);
    UIGraphRefresh(&referee_info.referee_id, 5, UI_Circle_t[0], UI_Circle_t[1], UI_Circle_t[2],UI_Circle_t [3], UI_Circle_t[4]);
    UIGraphRefresh(&referee_info.referee_id, 1,UI_Rectangle[0]);


    }
    else {
        UIIntDraw(&UI_Number_t[0],"sm0",UI_Graph_Change,9,UI_Color_White,20,3,300,700,ui_cmd_recv.auto_confirm_flag);
        UIIntDraw(&UI_Number_t[1],"sm1",UI_Graph_Change,9,UI_Color_White,20,3,300+100,700,ui_cmd_recv.auto_decide_flag);

        switch (ui_cmd_recv.PC_Mode){
            case 0:{
                UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_Pink, 10, UI_Circle_t[1].start_x, UI_Circle_t[1].start_y, 15);
                UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[2].start_x, UI_Circle_t[2].start_y, 15);
                UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[3].start_x, UI_Circle_t[3].start_y, 15);
                UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[4].start_x , UI_Circle_t[4].start_y, 15);
                break;
            }
            case 1:{
                UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[1].start_x, UI_Circle_t[1].start_y, 15);
                UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_Pink, 10, UI_Circle_t[2].start_x, UI_Circle_t[2].start_y, 15);
                UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[3].start_x, UI_Circle_t[3].start_y, 15);
                UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[4].start_x , UI_Circle_t[4].start_y, 15);
                break;
            }
            case 2:{
                UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[1].start_x, UI_Circle_t[1].start_y, 15);
                UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[2].start_x, UI_Circle_t[2].start_y, 15);
                UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_Pink, 10, UI_Circle_t[3].start_x, UI_Circle_t[3].start_y, 15);
                UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[4].start_x, UI_Circle_t[4].start_y, 15);
                break;
            }
            case 3:{
                UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[1].start_x, UI_Circle_t[1].start_y, 15);
                UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[2].start_x, UI_Circle_t[2].start_y, 15);
                UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[3].start_x, UI_Circle_t[3].start_y, 15);
                UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_Pink, 10, UI_Circle_t[4].start_x , UI_Circle_t[4].start_y, 15);
                break;
            }
            default :break;
        }
        

        UIGraphRefresh(&referee_info.referee_id, 2, UI_Number_t[0],UI_Number_t[1]);
        UIGraphRefresh(&referee_info.referee_id, 5, UI_Circle_t[0], UI_Circle_t[1], UI_Circle_t[2],UI_Circle_t [3], UI_Circle_t[4]);
    }
}
void BuzzerTask(void *argument)
{
    for (;;) {
        if (flag_referee_init){
            My_UIGraphRefresh();
        }
        uiTask();
		osDelay(50);
    }
}