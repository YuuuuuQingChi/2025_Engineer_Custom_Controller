

#include "referee_init.h"
#include "referee_UI.h"
#include "referee_protocol.h"
#include "message_center.h"
#include "rm_referee.h"
#include "UI_ref.h"
#include "UI.h"
#include <string.h>
//#include "flashtask.h"
#include "UI_interface.h"
//#include "UI_user_defined.h"

float UI_debug_value[7];

referee_info_t *referee_data;

UI_data_t UI_data_recv;
Subscriber_t *UI_cmd_sub;

uint8_t UI_Seq = 0;




/*正常模式下UI*/


static UI_GRAPH_INSTANCE* circle_main_pump;
static UI_GRAPH_INSTANCE* circle_mine_pump;
static UI_GRAPH_INSTANCE* circle_vision_pump;
static UI_GRAPH_INSTANCE* number_big_pitch_temp;



static UI_STRING_INSTANCE* string_main_pump;
static UI_STRING_INSTANCE* string_mine_pump;
static UI_STRING_INSTANCE* string_speed;
static UI_STRING_INSTANCE* string_vision;
static UI_STRING_INSTANCE* string_auto_mode;
static UI_STRING_INSTANCE* string_customer_mode;
static UI_STRING_INSTANCE* string_temp_big_pitch;
static UI_STRING_INSTANCE* string_custom;
static UI_STRING_INSTANCE* string_auto;
static UI_STRING_INSTANCE* string_chassis_speed;







struct{
    uint32_t pos_x;
    uint32_t pos_y;
    float height;
}height_makerLine;
struct{
    uint32_t pos_x;
    uint32_t pos_y;
    uint32_t dx;
    uint32_t dy;
    uint32_t width;
    int32_t offset_angle;
}armour_maker;
void get_referee_data(referee_info_t *referee_data)
{
    referee_data                               = referee_data;
    referee_data->referee_id.Robot_Color       = referee_data->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_data->referee_id.Robot_ID          = referee_data->GameRobotState.robot_id;
    referee_data->referee_id.Cilent_ID         = 0x0100 + referee_data->referee_id.Robot_ID; // 计算客户端ID
    referee_data->referee_id.Receiver_Robot_ID = 0;
}

static void ui_reload(){
    /*自定义UI*/
    // UserDefinedUI_init();
    /*UI注册-正常*/
    //圆
    
    circle_main_pump = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,35,650,820,15);
    circle_mine_pump = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,35,650,745,15);
    circle_vision_pump = UI_Graph_Init(GraphType_Round, 2, 1,Graphic_Color_White,35,650,670,15);

  
    //静态字符串
    string_main_pump = UI_String_Init(0,2,Graphic_Color_Yellow,35,3,290,840,"MAIN_PUMP:");
    string_mine_pump = UI_String_Init(0,2,Graphic_Color_Yellow,35,3,290,765,"MINE_PUMP:");
    string_speed = UI_String_Init(0,2,Graphic_Color_Yellow,40,3,700,200,"SPEED:");
    string_vision = UI_String_Init(0,2,Graphic_Color_Yellow,35,3,290,690,"VISION:");
    string_auto_mode = UI_String_Init(0,2,Graphic_Color_Yellow,35,3,1290,840,"AUTO:");
    string_customer_mode = UI_String_Init(0,2,Graphic_Color_Yellow,35,3,1290,765,"CUSTOME:");
    string_temp_big_pitch = UI_String_Init(0,2,Graphic_Color_Yellow,35,3,1290,690,"BIG_PITCH_T");


    //动态字符串    
    string_auto = UI_String_Init(0,3,Graphic_Color_Pink,35,3,1450,840,"NONE");
    string_custom = UI_String_Init(0,3,Graphic_Color_Pink,35,3,1550,765,"NONE");
    string_chassis_speed = UI_String_Init(0,3,Graphic_Color_Pink,40,3,920,200,"FAST");

    number_big_pitch_temp = UI_Graph_Init(GraphType_Number, 0, 5, Graphic_Color_White, 4, 1680, 690, 30);

    //清除所有UI
    UIDelete(&referee_data->referee_id, UI_Data_Del_ALL, 0);
}

void MyUIInit(void)
{
    /*数据初始化*/
    height_makerLine.pos_x = 1230;
    height_makerLine.pos_y = 430;
    armour_maker.pos_x = 960;armour_maker.pos_y = 540;
    armour_maker.dx = 100;armour_maker.dy = 100;armour_maker.width = 10;

    /*裁判系统初始化*/
    referee_data = RefereeHardwareInit(&huart10);
    
    osDelay(200);

    get_referee_data(referee_data);

    UI_cmd_sub = SubRegister("UI",sizeof(UI_data_t));

    ui_reload();
}

static void UI_operate()
    {   
        
        // 主气泵
        UI_StateSwitchDetect_Graph(circle_main_pump, 3, UI_data_recv.main_air_flag, Graphic_Color_White,Graphic_Color_Pink,Graphic_Color_White );
        //矿仓气泵
        UI_StateSwitchDetect_Graph(circle_mine_pump, 3, UI_data_recv.mine_air_flag, Graphic_Color_White,Graphic_Color_Pink,Graphic_Color_White );
        //自定义控制器的连接
        UI_StateSwitchDetect_Graph(circle_vision_pump, 2, UI_data_recv.custom_connect, Graphic_Color_White,Graphic_Color_Pink);
        //自定义控制器是否在使用
        UI_StringSwitchDetect_Char(string_custom, 2, UI_data_recv.Control_mode,"YES","NONE");
        UI_ColorSwitchDetect_Char(string_custom, 2, UI_data_recv.Control_mode,Graphic_Color_Pink,Graphic_Color_White);
        //自动模式的种类
        UI_StringSwitchDetect_Char(string_auto, 11, UI_data_recv.auto_type,"NONE","MID_GOLD","LEFT_GOLD","RIGHT_GOLD","PULL_GOLD","RESET","RECEIVE_MINE","GIVE_MINE","NO_TURTLE","ONE_TURTLE","DOUBLE_TURTLE");
        UI_ColorSwitchDetect_Char(string_auto, 2, UI_data_recv.Control_mode,Graphic_Color_White,Graphic_Color_Pink);
        //大pitch的温度
        number_big_pitch_temp->param.Number.value = UI_data_recv.big_pitch_temputure;
        number_big_pitch_temp->color =UI_data_recv.big_pitch_temputure < 75 ? Graphic_Color_White : Graphic_Color_Pink;
        UI_StringSwitchDetect_Char(string_chassis_speed,3,UI_data_recv.chassis_speed,"FAST","MID","LOW");



    
}
void MyUIRefresh(void)
{
    SubGetMessage(UI_cmd_sub, &UI_data_recv);
    
    static uint8_t debug_flag_switch = 0;
    
        UI_operate();
    
    if(UI_data_recv.control_refresh == 1)
    {
        UIDelete(&referee_data->referee_id, 0, 0);
        UI_String_Refresh();
        
    }
    UI_String_Refresh();
    UI_Graph_Refresh();
}

void UI_TASK(void *argument)
{
  /* USER CODE BEGIN UI_TASK */
  /* Infinite loop */
    UNUSED(argument);
    MyUIInit();
    osDelay(300);
    for(;;)
  {
       MyUIRefresh();
       osDelay(1);
    }

}