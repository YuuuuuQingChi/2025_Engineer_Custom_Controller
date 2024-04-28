// #ifndef D19E1F6E_B4C3_41CB_80F0_D289EC7CAB79
// #define D19E1F6E_B4C3_41CB_80F0_D289EC7CAB79
// #ifndef UI_TASK_H_
// #define UI_TASK_H_
// #endif
// #endif /* D19E1F6E_B4C3_41CB_80F0_D289EC7CAB79 */

// extern char Send_Once_Flag;//初始化标志
// //屏幕宽度
// #define SCREEN_WIDTH 1080
// //屏幕长度
// #define SCREEN_LENGTH 1920

// #define REFEREE_SOF 0xA5 // 起始字节,协议固定为0xA5
// #define Robot_Red 0
// #define Robot_Blue 1
// #define Communicate_Data_LEN 5 
// #pragma pack(1)
// /* 图形配置参数__图形操作 */
// typedef enum
// {
// 	UI_Graph_ADD = 1,
// 	UI_Graph_Change = 2,
// 	UI_Graph_Del = 3,
// } UI_Graph_Operate_e;
// ///* 交互数据ID */
// typedef enum
// {
// 	UI_Data_ID_Del = 0x100,
// 	UI_Data_ID_Draw1 = 0x101,
// 	UI_Data_ID_Draw2 = 0x102,
// 	UI_Data_ID_Draw5 = 0x103,
// 	UI_Data_ID_Draw7 = 0x104,
// 	UI_Data_ID_DrawChar = 0x110,

// 	/* 自定义交互数据部分 */
// 	Communicate_Data_ID = 0x0200,

// } Interactive_Data_ID_e;
// /* 图形配置参数__图形类型 */
// typedef enum
// {
// 	UI_Graph_Line = 0,		// 直线
// 	UI_Graph_Rectangle = 1, // 矩形
// 	UI_Graph_Circle = 2,	// 整圆
// 	UI_Graph_Ellipse = 3,	// 椭圆
// 	UI_Graph_Arc = 4,		// 圆弧
// 	UI_Graph_Float = 5,		// 浮点型
// 	UI_Graph_Int = 6,		// 整形
// 	UI_Graph_Char = 7,		// 字符型

// } UI_Graph_Type_e;
// /* 交互数据长度 */
// typedef enum
// {
// 	Interactive_Data_LEN_Head = 6,
// 	UI_Operate_LEN_Del = 2,
// 	UI_Operate_LEN_PerDraw = 15,
// 	UI_Operate_LEN_DrawChar = 15 + 30,

// 	/* 自定义交互数据部分 */
// 	// Communicate_Data_LEN = 5,

// } Interactive_Data_Length_e;
// /* 图形配置参数__图形颜色 */
// typedef enum
// {
// 	UI_Color_Main = 0, // 红蓝主色
// 	UI_Color_Yellow = 1,
// 	UI_Color_Green = 2,
// 	UI_Color_Orange = 3,
// 	UI_Color_Purplish_red = 4, // 紫红色
// 	UI_Color_Pink = 5,
// 	UI_Color_Cyan = 6, // 青色
// 	UI_Color_Black = 7,
// 	UI_Color_White = 8,
// } UI_Graph_Color_e;

// /* 通信协议长度 */
// typedef enum
// {
// 	LEN_HEADER = 5, // 帧头长
// 	LEN_CMDID = 2,	// 命令码长度
// 	LEN_TAIL = 2,	// 帧尾CRC16

// 	LEN_CRC8 = 4, // 帧头CRC8校验长度=帧头+数据长+包序号
// } JudgeFrameLength_e;


// /* 命令码ID,用来判断接收的是什么数据 */
// typedef enum
// {
// 	ID_game_state = 0x0001,				   // 比赛状态数据
// 	ID_game_result = 0x0002,			   // 比赛结果数据
// 	ID_game_robot_survivors = 0x0003,	   // 比赛机器人血量数据
// 	ID_event_data = 0x0101,				   // 场地事件数据
// 	ID_supply_projectile_action = 0x0102,  // 场地补给站动作标识数据
// 	ID_supply_projectile_booking = 0x0103, // 场地补给站预约子弹数据
// 	ID_game_robot_state = 0x0201,		   // 机器人状态数据
// 	ID_power_heat_data = 0x0202,		   // 实时功率热量数据
// 	ID_game_robot_pos = 0x0203,			   // 机器人位置数据
// 	ID_buff_musk = 0x0204,				   // 机器人增益数据
// 	ID_aerial_robot_energy = 0x0205,	   // 空中机器人能量状态数据
// 	ID_robot_hurt = 0x0206,				   // 伤害状态数据
// 	ID_shoot_data = 0x0207,				   // 实时射击数据
// 	ID_student_interactive = 0x0301,	   // 机器人间交互数据
// } CmdID_e;
// /* 帧头定义 */
// typedef struct
// {
// 	uint8_t SOF;
// 	uint16_t DataLength;
// 	uint8_t Seq;
// 	uint8_t CRC8;
// } xFrameHeader;

// typedef struct
// {
//    uint8_t Robot_Color;        // 机器人颜色
//    uint16_t Robot_ID;          // 本机器人ID
//    uint16_t Cilent_ID;         // 本机器人对应的客户端ID
//    uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
// } referee_id_t;

// // 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
// typedef struct
// {
//    referee_id_t referee_id;
// } referee_info_t;


// /* 交互数据头结构 */
// typedef struct
// {
// 	uint16_t data_cmd_id; // 由于存在多个内容 ID，但整个cmd_id 上行频率最大为 10Hz，请合理安排带宽。注意交互部分的上行频率
// 	uint16_t sender_ID;
// 	uint16_t receiver_ID;
// } ext_student_interactive_header_data_t;
// /* 图形数据 */
// typedef struct
// {
// 	uint8_t graphic_name[3];
// 	uint32_t operate_tpye : 3;
// 	uint32_t graphic_tpye : 3;
// 	uint32_t layer : 4;
// 	uint32_t color : 4;
// 	uint32_t start_angle : 9;
// 	uint32_t end_angle : 9;
// 	uint32_t width : 10;
// 	uint32_t start_x : 11;
// 	uint32_t start_y : 11;
// 	uint32_t radius : 10;
// 	uint32_t end_x : 11;
// 	uint32_t end_y : 11;
// } Graph_Data_t;
// typedef struct
// {
// 	Graph_Data_t Graph_Control;
// 	uint8_t show_Data[30];
// } String_Data_t; // 打印字符串数据
// typedef struct
// /* 此处的定义只与UI绘制有关 */
// {
//    xFrameHeader FrameHeader;
//    uint16_t CmdID;
//    ext_student_interactive_header_data_t datahead;
//    uint8_t Delete_Operate; // 删除操作
//    uint8_t Layer;
//    uint16_t frametail;
// } UI_delete_t;
// /* 删除操作 */
// typedef enum
// {
// 	UI_Data_Del_NoOperate = 0,
// 	UI_Data_Del_Layer = 1,
// 	UI_Data_Del_ALL = 2, // 删除全部图层，后面的参数已经不重要了。
// } UI_Delete_Operate_e;



// typedef struct
// {
//    xFrameHeader FrameHeader;
//    uint16_t CmdID;
//    ext_student_interactive_header_data_t datahead;
//    uint16_t frametail;
// } UI_GraphReFresh_t;

// typedef struct
// {
//    xFrameHeader FrameHeader;
//    uint16_t CmdID;
//    ext_student_interactive_header_data_t datahead;
//    String_Data_t String_Data;
//    uint16_t frametail;
// } UI_CharReFresh_t; // 打印字符串数据

// #pragma pack()


// void My_UI_Refresh();

