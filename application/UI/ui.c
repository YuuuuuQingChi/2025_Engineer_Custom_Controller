#include "ui.h"
#include "referee_UI.h"
#include "message_center.h"
#include "cmsis_os.h"
#include "robot_cmd.h"
#include "robot.h"
#include "robot_cmd.h"
#include "remote_control.h"
extern int flag_refresh_ui;
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

extern PC_Mode_t PC_Mode;

void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_info.referee_id.Robot_Color       = referee_info.GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_info.referee_id.Cilent_ID         = 0x0100 + referee_info.GameRobotState.robot_id; // 计算客户端ID
    referee_info.referee_id.Robot_ID          = referee_info.GameRobotState.robot_id;          // 计算机器人ID
    referee_info.referee_id.Receiver_Robot_ID = 0;
}

void My_UIGraphInit(){
    // UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 9);
    // UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 8);
    // UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 7);
    // UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 6);

    sprintf(UI_State_sta[4].show_Data, "Sucker");
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 100, 700, "Sucker");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[4]);

}

void My_UIGraphRefresh()
{
    DeterminRobotID();
    if (flag_refresh_ui){
        flag_refresh_ui=0;
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 9);
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 8);
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 7);
	
     UICircleDraw(&UI_Circle_t[0], "sc0", UI_Graph_ADD, 9, UI_Color_White, 10, 120,650, 8);
     UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_ADD, 9, UI_Color_White, 10, 600, 130, 8);
     UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_ADD, 9, UI_Color_White, 10, 650, 130, 8);
     UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_ADD, 9, UI_Color_White, 10, 700, 130, 8);
     UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_ADD, 9, UI_Color_White, 10, 750, 130, 8);
    //  UICircleDraw(&UI_Circle_t[5], "sc4", UI_Graph_ADD, 9, UI_Color_White, 10, 550 , 130, 8);
    
    sprintf(UI_State_sta[4].show_Data, "Sucker");
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 9, UI_Color_Yellow, 12, 2, 100, 700, "Sucker");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[4]);

    UIIntDraw(&UI_Number_t[0],"sm0",UI_Graph_ADD,9,UI_Color_White,20,3,300,700,auto_confirm_flag);
    UIIntDraw(&UI_Number_t[1],"sm1",UI_Graph_ADD,9,UI_Color_White,20,3,300,700,auto_decide_flag);
    // UIFloatDraw(&UI_Number_t[1], "sm1", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 5, 1, 300 + 100, 700,auto_decide_flag);
    UIGraphRefresh(&referee_info.referee_id, 5, UI_Deriction_line[0], UI_Deriction_line[1], UI_Deriction_line[2], UI_Deriction_line[3], UI_Circle_t[0]);
    UIGraphRefresh(&referee_info.referee_id, 2, UI_Number_t[0],UI_Number_t[1]);
    UIGraphRefresh(&referee_info.referee_id, 5, UI_Circle_t[0], UI_Circle_t[1], UI_Circle_t[2],UI_Circle_t [3], UI_Circle_t[4]);


    }
    else {
        UIIntDraw(&UI_Number_t[0],"sm0",UI_Graph_Change,9,UI_Color_White,20,3,300,700,auto_confirm_flag);
        UIIntDraw(&UI_Number_t[1],"sm1",UI_Graph_Change,9,UI_Color_White,20,3,300+100,700,auto_decide_flag);

        switch (PC_Mode){
            case 0:{
                UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_Pink, 10, 600, 130, 8);
                UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 10, 650, 130, 8);
                UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_White, 10, 700, 130, 8);
                UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_White, 10, 750 , 130, 8);
                break;
            }
            case 1:{
                UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, 600, 130, 8);
                UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_Pink, 10, 650, 130, 8);
                UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_White, 10, 700, 130, 8);
                UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_White, 10, 750 , 130, 8);
                break;
            }
            case 2:{
                UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, 600, 130, 8);
                UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 10, 650, 130, 8);
                UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_Pink, 10, 700, 130, 8);
                UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_White, 10, 750 , 130, 8);
                break;
            }
            case 3:{
                UICircleDraw(&UI_Circle_t[1], "sc1", UI_Graph_Change, 9, UI_Color_White, 10, 600, 130, 8);
                UICircleDraw(&UI_Circle_t[2], "sc2", UI_Graph_Change, 9, UI_Color_White, 10, 650, 130, 8);
                UICircleDraw(&UI_Circle_t[3], "sc3", UI_Graph_Change, 9, UI_Color_White, 10, 700, 130, 8);
                UICircleDraw(&UI_Circle_t[4], "sc4", UI_Graph_Change, 9, UI_Color_Pink, 10, 750 , 130, 8);
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
		osDelay(50);
    }
}