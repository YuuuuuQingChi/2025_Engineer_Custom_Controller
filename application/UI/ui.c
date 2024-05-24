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
    UI_Circle_t[1].start_x=650;
    UI_Circle_t[2].start_x=800;
    UI_Circle_t[3].start_x=950;
    UI_Circle_t[4].start_x=230;
    UI_Circle_t[5].start_x=230;

    UI_Circle_t[0].start_y=650;
    UI_Circle_t[1].start_y=130;
    UI_Circle_t[2].start_y=130;
    UI_Circle_t[3].start_y=130;
    UI_Circle_t[4].start_y=760;

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
    uint32_t circle_x=340;
    uint32_t circle_y=760; 
    uint32_t string_x=15;
    uint32_t string_y=790;
    uint32_t rectangle_x[3]={0};
    rectangle_x[0]=650;
    rectangle_x[1]=800;
    rectangle_x[2]=950;
    uint32_t rectangle_y=130;

    if (ui_cmd_recv.flag_refresh_ui){ //静态部分，不自动刷新
        ui_feedback_data.flag_refresh_ui=0;

    //刷新时先删UI
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 9);
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 8);
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 7);
	
    //UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_ADD, 9, UI_Color_White, 10, UI_Circle_t[0],UI_Circle_t[0].start_y, 8);
    //模式指示灯 
    // UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_ADD, 9, UI_Color_White, 10, UI_Circle_t[1],UI_Circle_t[1].start_y, 15);
    // UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_ADD, 9, UI_Color_White, 10, UI_Circle_t[2],UI_Circle_t[2].start_y, 15);
    // UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_ADD, 9, UI_Color_White, 10, UI_Circle_t[3],UI_Circle_t[3].start_y, 15);
    // UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_ADD, 9, UI_Color_White, 10, UI_Circle_t[4],UI_Circle_t[4].start_y, 15);
    // //  UICircleDraw(&UI_Circle_t[5], "sc4", UI_Graph_ADD, 9, UI_Color_White, 10, 550 , 130, 8);
    // UILineDraw(&UI_Deriction_line[0],"sl0",UI_Graph_ADD,9,UI_Color_Green,2,600,130,700,400);
    // UILineDraw(&UI_Deriction_line[1],"sl1",UI_Graph_ADD,9,UI_Color_Green,2,1920-600,130,1220,400);
    // UIRectangleDraw(&UI_Rectangle[0],"sj0",UI_Graph_ADD,9,UI_Color_Green,3,700,400,1220,700);
    
    //2气缸 3气泵 指示灯
    UICircleDraw(&UI_Circle_t[5], "sc5", UI_Graph_ADD, 9, UI_Color_White, 10, circle_x,circle_y+20, 20);
    // UICircleDraw(&UI_Circle_t[6], "sc6", UI_Graph_ADD, 9, UI_Color_White, 10, circle_x,circle_y-40, 20);
    // UICircleDraw(&UI_Circle_t[7], "sc7", UI_Graph_ADD, 9, UI_Color_White, 10, circle_x,circle_y-100, 20);
    // UICircleDraw(&UI_Circle_t[8], "sc8", UI_Graph_ADD, 9, UI_Color_White, 10, circle_x,circle_y-160, 20);
    // UICircleDraw(&UI_Circle_t[9], "sc9", UI_Graph_ADD, 9, UI_Color_White, 10, circle_x,circle_y-220, 20);

    sprintf(UI_State_sta[0].show_Data, "MAIN AIR");
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 9, UI_Color_Yellow, 30, 2, string_x, string_y+5, "MAIN AIR");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[0]);

    // sprintf(UI_State_sta[1].show_Data, "LEFT AIR");
    // UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 9, UI_Color_Yellow, 30, 2, string_x,string_y-55, "LEFT AIR");
    // UICharRefresh(&referee_info.referee_id, UI_State_sta[1]);

    // sprintf(UI_State_sta[2].show_Data, "RIGHT AIR");
    // UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 9, UI_Color_Yellow, 30, 2, string_x,string_y-115, "RIGHT AIR");
    // UICharRefresh(&referee_info.referee_id, UI_State_sta[2]);

    // sprintf(UI_State_sta[3].show_Data, "UP GANG");
    // UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 9, UI_Color_Yellow, 30, 2, string_x,string_y-175, "UP GANG");
    // UICharRefresh(&referee_info.referee_id, UI_State_sta[3]);

    // sprintf(UI_State_sta[4].show_Data, "DOWN GANG");
    // UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 9, UI_Color_Yellow, 30, 2, string_x,string_y-235, "DOWN GANG");
    // UICharRefresh(&referee_info.referee_id, UI_State_sta[4]);
    
    //模式切换
    sprintf(UI_State_sta[5].show_Data, "MONY");
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 9, UI_Color_Yellow, 30, 2, UI_Circle_t[1].start_x,UI_Circle_t[1].start_y, "MONY");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[5]);

    sprintf(UI_State_sta[6].show_Data, "AUTO");
    UICharDraw(&UI_State_sta[6], "ss6", UI_Graph_ADD, 9, UI_Color_Yellow, 30, 2, UI_Circle_t[2].start_x,UI_Circle_t[2].start_y, "AUTO");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[6]);

    // sprintf(UI_State_sta[7].show_Data, "AUTO");
    // UICharDraw(&UI_State_sta[7], "ss7", UI_Graph_ADD, 9, UI_Color_Yellow, 30, 2, UI_Circle_t[3].start_x,UI_Circle_t[3].start_y, "AUTO");
    // UICharRefresh(&referee_info.referee_id, UI_State_sta[7]);
    
    UIRectangleDraw(&UI_Rectangle[1],"sj1",UI_Graph_ADD,9,UI_Color_Purplish_red,4,rectangle_x[ui_cmd_recv.PC_Mode-1]-20,rectangle_y-35,rectangle_x[ui_cmd_recv.PC_Mode-1]+120,rectangle_y+10);

    UIIntDraw(&UI_Number_t[0],"sm0",UI_Graph_ADD,9,UI_Color_White,20,3,250,string_y,ui_cmd_recv.auto_confirm_flag);
    UIIntDraw(&UI_Number_t[1],"sm1",UI_Graph_ADD,9,UI_Color_White,20,3,250,string_y-50,ui_cmd_recv.auto_decide_flag);   

    //刷新ui
    //UIGraphRefresh(&referee_info.referee_id, 5, UI_Deriction_line[0], UI_Deriction_line[1], UI_Deriction_line[2], UI_Deriction_line[3], UI_Circle_t[0]);
    UIGraphRefresh(&referee_info.referee_id, 2, UI_Number_t[0],UI_Number_t[1]);
    //UIGraphRefresh(&referee_info.referee_id, 5, UI_Circle_t[0], UI_Circle_t[1], UI_Circle_t[2],UI_Circle_t [3], UI_Circle_t[4]);
    // UIGraphRefresh(&referee_info.referee_id, 1,UI_Rectangle[0]);
    UIGraphRefresh(&referee_info.referee_id, 2, UI_Circle_t[5],  UI_Circle_t[7],UI_Circle_t [8], UI_Circle_t[9]);
    UIGraphRefresh(&referee_info.referee_id, 1, UI_Rectangle[1]);
    }
    else {
        UIIntDraw(&UI_Number_t[0],"sm0",UI_Graph_Change,9,UI_Color_White,35,5,500,700,ui_cmd_recv.auto_confirm_flag);
        UIIntDraw(&UI_Number_t[1],"sm1",UI_Graph_Change,9,UI_Color_White,35,5,500+100,700,ui_cmd_recv.auto_decide_flag);
        

        // switch (ui_cmd_recv.PC_Mode){
        //     case 1:{
        //         UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[1], UI_Circle_t[1].start_y, 15);
        //         UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_Pink, 10, UI_Circle_t[2], UI_Circle_t[2].start_y, 15);
        //         UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[3], UI_Circle_t[3].start_y, 15);
        //         UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[4] , UI_Circle_t[4].start_y, 15);
        //         break;
        //     }
        //     case 2:{
        //         UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[1], UI_Circle_t[1].start_y, 15);
        //         UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[2], UI_Circle_t[2].start_y, 15);
        //         UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_Pink, 10, UI_Circle_t[3], UI_Circle_t[3].start_y, 15);
        //         UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[4], UI_Circle_t[4].start_y, 15);
        //         break;
        //     }
        //     case 3:{
        //         UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[1], UI_Circle_t[1].start_y, 15);
        //         UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[2], UI_Circle_t[2].start_y, 15);
        //         UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_White, 10, UI_Circle_t[3], UI_Circle_t[3].start_y, 15);
        //         UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_Pink, 10, UI_Circle_t[4] , UI_Circle_t[4].start_y, 15);
        //         break;
        //     }
        //     default :break;
        //}

        UICircleDraw(&UI_Circle_t[5], "sc5", UI_Graph_Change, 9, ui_cmd_recv.main_air_flag==1?5:8, 10, circle_x,circle_y+20, 20);
        // UICircleDraw(&UI_Circle_t[6], "sc6", UI_Graph_Change, 9, ui_cmd_recv.left_air_flag==1?5:8, 10, circle_x,circle_y-40, 20);
        // UICircleDraw(&UI_Circle_t[7], "sc7", UI_Graph_Change, 9, ui_cmd_recv.right_air_flag==1?5:8, 10,circle_x,circle_y-100, 20);
        // UICircleDraw(&UI_Circle_t[8], "sc8", UI_Graph_Change, 9, ui_cmd_recv.air_up_gang_flag==1?5:8, 10, circle_x,circle_y-160, 20);
        // UICircleDraw(&UI_Circle_t[9], "sc9", UI_Graph_Change, 9, ui_cmd_recv.air_down_gang_flag==1?5:8, 10, circle_x,circle_y-220, 20);

        UIRectangleDraw(&UI_Rectangle[1],"sj1",UI_Graph_Change,9,UI_Color_Purplish_red,4,rectangle_x[ui_cmd_recv.PC_Mode-1]-20,rectangle_y-35,rectangle_x[ui_cmd_recv.PC_Mode-1]+120,rectangle_y+10);
        
        UIGraphRefresh(&referee_info.referee_id, 2, UI_Number_t[0],UI_Number_t[1]);
        //UIGraphRefresh(&referee_info.referee_id, 5, UI_Circle_t[0], UI_Circle_t[1], UI_Circle_t[2],UI_Circle_t [3], UI_Circle_t[4]);
        UIGraphRefresh(&referee_info.referee_id, 5, UI_Circle_t[5], UI_Circle_t[6], UI_Circle_t[7],UI_Circle_t [8], UI_Circle_t[9]);
        UIGraphRefresh(&referee_info.referee_id, 1, UI_Rectangle[1]);
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