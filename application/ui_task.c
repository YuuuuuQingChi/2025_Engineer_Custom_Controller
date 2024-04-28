#include "ui_task.h"

// /**
//  *@Function:		ui_task(void *p_arg)
//  *@Description:	UI任务
//  *@Param:       形参
//  *@Return:	  	返回值
//  */
// void ui_task(void *p_arg)
// {

// 	while (1)
// 	{
// 		// 绘制UI
// 		//Show_UI();
// 		My_UI_Refresh();
// 		// 带宽30Hz
// 		osDelay(20);
// 	}
// }


// /**
//  * @file referee_UI.C
//  * @author kidneygood (you@domain.com)
//  * @brief
//  * @version 0.1
//  * @date 2023-1-18
//  *
//  * @copyright Copyright (c) 2022
//  *
//  */
// #include "referee_UI.h"

// /*-------------------------------------- �������� -----------------------------------------*/

// static referee_info_t referee_info;						  // 机器人相关信息
// static referee_info_t *referee_recv_info = &referee_info; // 机器人相关信息指针
// uint8_t UI_Seq;											  // ui数量

// static char Char_Engineer_Control_Mode[30];						 // 工程控制模式字符
// static char Char_Engineer_Air_Get_Mine_Motor_Mode[20];			 // 工程翻转电机模式字符
// static char Char_Engineer_Air_Switch_Mode[20];					 // 工程气泵开关模式字符
// static char Char_Engineer_Obstacle_Block_Switch_Mode[20];		 // 工程障碍块开关模式字符
// static char Char_Engineer_Duct_Switch_Mode[20];					 // 涵道开关模式字符
// static char Char_Engineer_Mine_Warehouse_Expand_Switch_Mode[20]; // 矿仓拓展模块状态字符
// static char Char_Engineer_Mine_Warehouse_Expand_Lock_Mode[20];	 // 矿仓固定模块状态字符
// static char Char_Engineer_Air_Switch_Dirction_Mode[20];			 // 矿仓吸盘转换方向状态字符

// static String_Data_t UI_Engineer_Mode_String[8] = {0}; // 工程模式ui（8种状态）

// static Graph_Data_t UI_Enginer_Air_Mine_Line[2] = {0}; // 空接位置线条

// static Graph_Data_t UI_Enginer_Obstacle_Block_Line[2] = {0}; // 障碍块位置线条

// /*-------------------------------------- �������� ----------------------------------------*/

// void RefereeSend(uint8_t *send, uint16_t tx_len);
// void Char_ReFresh(referee_id_t *_id, String_Data_t string_Data);
// /********************************************ɾ������*************************************
// **������_id ��Ӧ��id�ṹ��
// 		Del_Operate  ��Ӧͷ�ļ�ɾ������
// 		Del_Layer    Ҫɾ���Ĳ� ȡֵ0-9
// *****************************************************************************************/
// void UIDelete(referee_id_t *_id, uint8_t Del_Operate, uint8_t Del_Layer)
// {
// 	static UI_delete_t UI_delete_data;
// 	uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_Del; // ���㽻�����ݳ���

// 	UI_delete_data.FrameHeader.SOF = REFEREE_SOF;
// 	UI_delete_data.FrameHeader.DataLength = temp_datalength;
// 	UI_delete_data.FrameHeader.Seq = UI_Seq;
// 	UI_delete_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&UI_delete_data, LEN_CRC8, 0xFF);

// 	UI_delete_data.CmdID = ID_student_interactive;

// 	UI_delete_data.datahead.data_cmd_id = UI_Data_ID_Del;
// 	UI_delete_data.datahead.receiver_ID = _id->Cilent_ID;
// 	UI_delete_data.datahead.sender_ID = _id->Robot_ID;

// 	UI_delete_data.Delete_Operate = Del_Operate; // ɾ������
// 	UI_delete_data.Layer = Del_Layer;

// 	UI_delete_data.frametail = Get_CRC16_Check_Sum((uint8_t *)&UI_delete_data, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);
// 	/* ����0xFFFF,����crcУ�� */

// 	RefereeSend((uint8_t *)&UI_delete_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // ����

// 	UI_Seq++; // �����+1
// }
// /************************************************����ֱ��*************************************************
// **������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
// 		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
// 		Graph_Operate   ͼƬ��������ͷ�ļ�
// 		Graph_Layer    ͼ��0-9
// 		Graph_Color    ͼ����ɫ
// 		Graph_Width    ͼ���߿�
// 		Start_x��Start_y  ���xy����
// 		End_x��End_y   �յ�xy����
// **********************************************************************************************************/

// void Line_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
// 			   uint32_t Graph_Width, float Start_x, float Start_y, float End_x, float End_y)
// {
// 	int i;
// 	for (i = 0; i < 3 && graphname[i] != '\0'; i++) // �������0��Ϊֹ
// 	{
// 		graph->graphic_name[2 - i] = graphname[i]; // ���ڴ��ַ��������䣬���Ի���i��2-i
// 	}

// 	graph->operate_tpye = Graph_Operate;
// 	graph->graphic_tpye = UI_Graph_Line;
// 	graph->layer = Graph_Layer;
// 	graph->color = Graph_Color;

// 	graph->start_angle = 0;
// 	graph->end_angle = 0;
// 	graph->width = Graph_Width;
// 	graph->start_x = Start_x;
// 	graph->start_y = Start_y;
// 	graph->radius = 0;
// 	graph->end_x = End_x;
// 	graph->end_y = End_y;
// }

// /************************************************���ƾ���*************************************************
// **������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
// 		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
// 		Graph_Operate   ͼƬ��������ͷ�ļ�
// 		Graph_Layer    ͼ��0-9
// 		Graph_Color    ͼ����ɫ
// 		Graph_Width    ͼ���߿�
// 		Start_x��Start_y    ���xy����
// 		End_x��End_y        �ԽǶ���xy����
// **********************************************************************************************************/
// void Rectangle_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
// 					uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
// {
// 	int i;
// 	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
// 	{
// 		graph->graphic_name[2 - i] = graphname[i];
// 	}

// 	graph->graphic_tpye = UI_Graph_Rectangle;
// 	graph->operate_tpye = Graph_Operate;
// 	graph->layer = Graph_Layer;
// 	graph->color = Graph_Color;

// 	//	graph->start_angle = 0;
// 	//	graph->end_angle = 0;
// 	graph->width = Graph_Width;
// 	graph->start_x = Start_x;
// 	graph->start_y = Start_y;
// 	graph->radius = 0;
// 	graph->end_x = End_x;
// 	graph->end_y = End_y;
// }

// /************************************************������Բ*************************************************
// **������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
// 		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
// 		Graph_Operate   ͼƬ��������ͷ�ļ�
// 		Graph_Layer    ͼ��0-9
// 		Graph_Color    ͼ����ɫ
// 		Graph_Width    ͼ���߿�
// 		Start_x��Start_y    Բ��xy����
// 		Graph_Radius  Բ�ΰ뾶
// **********************************************************************************************************/

// void Circle_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
// 				 uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
// {
// 	int i;
// 	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
// 	{
// 		graph->graphic_name[2 - i] = graphname[i];
// 	}

// 	graph->graphic_tpye = UI_Graph_Circle;
// 	graph->operate_tpye = Graph_Operate;
// 	graph->layer = Graph_Layer;
// 	graph->color = Graph_Color;

// 	graph->start_angle = 0;
// 	graph->end_angle = 0;
// 	graph->width = Graph_Width;
// 	graph->start_x = Start_x;
// 	graph->start_y = Start_y;
// 	graph->radius = Graph_Radius;
// 	graph->end_x = 0;
// 	graph->end_y = 0;
// }
// /************************************************������Բ*************************************************
// **������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
// 		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
// 		Graph_Operate   ͼƬ��������ͷ�ļ�
// 		Graph_Layer    ͼ��0-9
// 		Graph_Color    ͼ����ɫ
// 		Graph_Width    ͼ���߿�
// 		Start_x��Start_y    Բ��xy����
// 		End_x��End_y        xy���᳤��
// **********************************************************************************************************/
// void Elliptical_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
// 					 uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y)
// {
// 	int i;
// 	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
// 	{
// 		graph->graphic_name[2 - i] = graphname[i];
// 	}

// 	graph->graphic_tpye = UI_Graph_Ellipse;
// 	graph->operate_tpye = Graph_Operate;
// 	graph->layer = Graph_Layer;
// 	graph->color = Graph_Color;
// 	graph->width = Graph_Width;

// 	graph->start_angle = 0;
// 	graph->end_angle = 0;
// 	graph->width = Graph_Width;
// 	graph->start_x = Start_x;
// 	graph->start_y = Start_y;
// 	graph->radius = 0;
// 	graph->end_x = end_x;
// 	graph->end_y = end_y;
// }

// /************************************************����Բ��*************************************************
// **������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
// 		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
// 		Graph_Operate   ͼƬ��������ͷ�ļ�
// 		Graph_Layer    ͼ��0-9
// 		Graph_Color    ͼ����ɫ
// 		Graph_StartAngle,Graph_EndAngle    ��ʼ��ֹ�Ƕ�
// 		Graph_Width    ͼ���߿�
// 		Start_y,Start_y    Բ��xy����
// 		x_Length,y_Length   xy���᳤��
// **********************************************************************************************************/

// void Arc_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
// 			  uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
// 			  uint32_t end_x, uint32_t end_y)
// {
// 	int i;
// 	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
// 	{
// 		graph->graphic_name[2 - i] = graphname[i];
// 	}

// 	graph->graphic_tpye = UI_Graph_Arc;
// 	graph->operate_tpye = Graph_Operate;
// 	graph->layer = Graph_Layer;
// 	graph->color = Graph_Color;

// 	graph->start_angle = Graph_StartAngle;
// 	graph->end_angle = Graph_EndAngle;
// 	graph->width = Graph_Width;
// 	graph->start_x = Start_x;
// 	graph->start_y = Start_y;
// 	graph->radius = 0;
// 	graph->end_x = end_x;
// 	graph->end_y = end_y;
// }

// /************************************************���Ƹ���������*************************************************
// **������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
// 		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
// 		Graph_Operate   ͼƬ��������ͷ�ļ�
// 		Graph_Layer    ͼ��0-9
// 		Graph_Color    ͼ����ɫ
// 		Graph_Size     �ֺ�
// 		Graph_Digit    С��λ��
// 		Graph_Width    ͼ���߿�
// 		Start_x��Start_y    ��ʼ����
// 		radius=a&0x3FF;   aΪ����������1000���32λ������
// 		end_x=(a>>10)&0x7FF;
// 		end_y=(a>>21)&0x7FF;
// **********************************************************************************************************/

// void Float_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
// 				uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float)
// {

// 	int i;
// 	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
// 	{
// 		graph->graphic_name[2 - i] = graphname[i];
// 	}
// 	graph->graphic_tpye = UI_Graph_Float;
// 	graph->operate_tpye = Graph_Operate;
// 	graph->layer = Graph_Layer;
// 	graph->color = Graph_Color;

// 	graph->width = Graph_Width;
// 	graph->start_x = Start_x;
// 	graph->start_y = Start_y;
// 	graph->start_angle = Graph_Size;
// 	graph->end_angle = Graph_Digit;

// 	graph->radius = Graph_Float & 0x3FF;
// 	graph->end_x = (Graph_Float >> 10) & 0x7FF;
// 	graph->end_y = (Graph_Float >> 21) & 0x7FF;
// }

// /************************************************������������*************************************************
// **������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
// 		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
// 		Graph_Operate   ͼƬ��������ͷ�ļ�
// 		Graph_Layer    ͼ��0-9
// 		Graph_Color    ͼ����ɫ
// 		Graph_Size     �ֺ�
// 		Graph_Width    ͼ���߿�
// 		Start_x��Start_y    ��ʼ����
// 		radius=a&0x3FF;   aΪ32λ������
// 		end_x=(a>>10)&0x7FF;
// 		end_y=(a>>21)&0x7FF;
// **********************************************************************************************************/
// void Integer_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
// 				  uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer)
// {
// 	int i;
// 	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
// 	{
// 		graph->graphic_name[2 - i] = graphname[i];
// 	}
// 	graph->graphic_tpye = UI_Graph_Int;
// 	graph->operate_tpye = Graph_Operate;
// 	graph->layer = Graph_Layer;
// 	graph->color = Graph_Color;

// 	graph->start_angle = Graph_Size;
// 	graph->end_angle = 0;
// 	graph->width = Graph_Width;
// 	graph->start_x = Start_x;
// 	graph->start_y = Start_y;
// 	graph->radius = Graph_Integer & 0x3FF;
// 	graph->end_x = (Graph_Integer >> 10) & 0x7FF;
// 	graph->end_y = (Graph_Integer >> 21) & 0x7FF;
// }

// /************************************************�����ַ�������*************************************************
// **������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
// 		graphname[3]   ͼƬ���ƣ����ڱ�ʶ����
// 		Graph_Operate   ͼƬ��������ͷ�ļ�
// 		Graph_Layer    ͼ��0-9
// 		Graph_Color    ͼ����ɫ
// 		Graph_Size     �ֺ�
// 		Graph_Width    ͼ���߿�
// 		Start_x��Start_y    ��ʼ����

// **������*graph Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
// 		fmt��Ҫ��ʾ���ַ���
// 		�˺�����ʵ�ֺ;���ʹ��������printf����
// **********************************************************************************************************/
// void Char_Draw(String_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
// 			   uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...)
// {
// 	int i;
// 	for (i = 0; i < 3 && graphname[i] != '\0'; i++)
// 	{
// 		graph->Graph_Control.graphic_name[2 - i] = graphname[i];
// 	}

// 	graph->Graph_Control.graphic_tpye = UI_Graph_Char;
// 	graph->Graph_Control.operate_tpye = Graph_Operate;
// 	graph->Graph_Control.layer = Graph_Layer;
// 	graph->Graph_Control.color = Graph_Color;

// 	graph->Graph_Control.width = Graph_Width;
// 	graph->Graph_Control.start_x = Start_x;
// 	graph->Graph_Control.start_y = Start_y;
// 	graph->Graph_Control.start_angle = Graph_Size;
// 	graph->Graph_Control.radius = 0;
// 	graph->Graph_Control.end_x = 0;
// 	graph->Graph_Control.end_y = 0;

// 	va_list ap;
// 	va_start(ap, fmt);
// 	vsprintf((char *)graph->show_Data, fmt, ap); // ʹ�ò����б����и�ʽ����������ַ���
// 	va_end(ap);
// 	graph->Graph_Control.end_angle = strlen((const char *)graph->show_Data);
// }

// /* UI���ͺ�����ʹ������Ч��
//    ������ cnt   ͼ�θ���
// 			...   ͼ�α�������
//    Tips�����ú���ֻ������1��2��5��7��ͼ�Σ�������ĿЭ��δ�漰
//  */
// void UI_ReFresh(referee_id_t *_id, int cnt, ...)
// {
// 	UI_GraphReFresh_t UI_GraphReFresh_data;
// 	Graph_Data_t graphData;

// 	uint8_t temp_datalength = LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * cnt + LEN_TAIL; // ���㽻�����ݳ���

// 	static uint8_t buffer[512]; // �������ݻ���

// 	va_list ap;		   // ����һ�� va_list ���ͱ���
// 	va_start(ap, cnt); // ��ʼ�� va_list ����Ϊһ�������б�

// 	UI_GraphReFresh_data.FrameHeader.SOF = REFEREE_SOF;
// 	UI_GraphReFresh_data.FrameHeader.DataLength = Interactive_Data_LEN_Head + cnt * UI_Operate_LEN_PerDraw;
// 	UI_GraphReFresh_data.FrameHeader.Seq = UI_Seq;
// 	UI_GraphReFresh_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&UI_GraphReFresh_data, LEN_CRC8, 0xFF);

// 	UI_GraphReFresh_data.CmdID = ID_student_interactive;

// 	switch (cnt)
// 	{
// 	case 1:
// 		UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw1;
// 		break;
// 	case 2:
// 		UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw2;
// 		break;
// 	case 5:
// 		UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw5;
// 		break;
// 	case 7:
// 		UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw7;
// 		break;
// 	case 0:
// 		UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_DrawChar;
// 		break;
// 	}

// 	UI_GraphReFresh_data.datahead.receiver_ID = _id->Cilent_ID;
// 	UI_GraphReFresh_data.datahead.sender_ID = _id->Robot_ID;
// 	memcpy(buffer, (uint8_t *)&UI_GraphReFresh_data, LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head); // ��֡ͷ�������롢��������֡ͷ�����ָ��Ƶ�������

// 	for (uint8_t i = 0; i < cnt; i++) // ���ͽ������ݵ�����֡��������CRC16У��ֵ
// 	{
// 		graphData = va_arg(ap, Graph_Data_t); // ���ʲ����б��е�ÿ����,�ڶ�����������Ҫ���صĲ���������,��ȡֵʱ��Ҫ����ǿ��ת��Ϊָ�����͵ı���
// 		memcpy(buffer + (LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * i), (uint8_t *)&graphData, UI_Operate_LEN_PerDraw);
// 	}
// 	Append_CRC16_Check_Sum(buffer, temp_datalength);
// 	RefereeSend(buffer, temp_datalength);

// 	va_end(ap); // �����ɱ�����Ļ�ȡ
// }

// /************************************************UI�����ַ���ʹ������Ч��*********************************/
// void Char_ReFresh(referee_id_t *_id, String_Data_t string_Data)
// {
// 	static UI_CharReFresh_t UI_CharReFresh_data;

// 	uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_DrawChar; // ���㽻�����ݳ���

// 	UI_CharReFresh_data.FrameHeader.SOF = REFEREE_SOF;
// 	UI_CharReFresh_data.FrameHeader.DataLength = temp_datalength;
// 	UI_CharReFresh_data.FrameHeader.Seq = UI_Seq;
// 	UI_CharReFresh_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&UI_CharReFresh_data, LEN_CRC8, 0xFF);

// 	UI_CharReFresh_data.CmdID = ID_student_interactive;

// 	UI_CharReFresh_data.datahead.data_cmd_id = UI_Data_ID_DrawChar;

// 	UI_CharReFresh_data.datahead.receiver_ID = _id->Cilent_ID;
// 	UI_CharReFresh_data.datahead.sender_ID = _id->Robot_ID;

// 	UI_CharReFresh_data.String_Data = string_Data;

// 	UI_CharReFresh_data.frametail = Get_CRC16_Check_Sum((uint8_t *)&UI_CharReFresh_data, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);

// 	RefereeSend((uint8_t *)&UI_CharReFresh_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // ����

// 	UI_Seq++; // �����+1
// }

// /**
//  * @brief  �жϸ���ID��ѡ��ͻ���ID
//  * @param  referee_info_t *referee_recv_info
//  * @retval none
//  * @attention
//  */
// static void DeterminRobotID()
// {
// 	// idС��7�Ǻ�ɫ,����7����ɫ,0Ϊ��ɫ��1Ϊ��ɫ   #define Robot_Red 0    #define Robot_Blue 1
// 	referee_recv_info->referee_id.Robot_Color = Referee_Inf.game_robot_state.robot_id > 7 ? Robot_Blue : Robot_Red;
// 	referee_recv_info->referee_id.Robot_ID = Referee_Inf.game_robot_state.robot_id;
// 	referee_recv_info->referee_id.Cilent_ID = 0x0100 + Referee_Inf.game_robot_state.robot_id; // ����ͻ���ID
// 	referee_recv_info->referee_id.Receiver_Robot_ID = 0;
// }

// /************************** ��ȡUI������Ҫ�Ļ�����״̬����*******************************************/

// char Send_Once_Flag = 0; // ��ʼ����־
// float change_yaw;		 // ����״̬
// float thert_x;
// float thert_y;
// u32 Rect_De[4] = {1540, 555, 1660, 645};
// float t;
// void My_UI_Refresh()
// {

// 	if (Send_Once_Flag == 0)
// 	{

// 		// while (Referee_Inf.game_robot_state.robot_id == 0);
// 		DeterminRobotID();
// 		// ���UI
// 		UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 9);
// 		UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 8);
// 		UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 7);
// 		UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 6);
// 		UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 5);
// 		// 空接位置线条
// 		Line_Draw(&UI_Enginer_Air_Mine_Line[0], "sl0", UI_Graph_ADD, 8, UI_Color_Cyan, 1, SCREEN_LENGTH / 2 - 100, 0, SCREEN_LENGTH / 2 - 100, SCREEN_WIDTH);
// 		Line_Draw(&UI_Enginer_Air_Mine_Line[1], "sl1", UI_Graph_ADD, 8, UI_Color_Cyan, 1, SCREEN_LENGTH / 2 + 100, 0, SCREEN_LENGTH / 2 + 100, SCREEN_WIDTH);

// 		// 障碍块位置线条
// 		Line_Draw(&UI_Enginer_Obstacle_Block_Line[0], "sl2", UI_Graph_ADD, 8, UI_Color_Cyan, 1, SCREEN_LENGTH / 2 - 100, 0, SCREEN_LENGTH / 2 - 120, 312);
// 		Line_Draw(&UI_Enginer_Obstacle_Block_Line[1], "sl3", UI_Graph_ADD, 8, UI_Color_Cyan, 1, SCREEN_LENGTH / 2 + 100, 0, SCREEN_LENGTH / 2 + 100, 312);

// 		// Line_Draw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_Cyan, 1, SCREEN_LENGTH / 2 - 12, SCREEN_WIDTH / 2, SCREEN_LENGTH / 2 - 12, SCREEN_WIDTH / 2 - 320);
// 		// Line_Draw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_Cyan, 1, SCREEN_LENGTH / 2 + 44, SCREEN_WIDTH / 2, SCREEN_LENGTH / 2 + 44, SCREEN_WIDTH / 2 - 320);
// 		// // ��������
// 		// Line_Draw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 320);
// 		// Line_Draw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2, SCREEN_LENGTH / 2 + 19.5, SCREEN_WIDTH / 2);
// 		// Line_Draw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 320, SCREEN_LENGTH / 2 + 19.5, SCREEN_WIDTH / 2 - 320);
// 		// // Line_Draw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 + 19.5, SCREEN_WIDTH / 2, 				SCREEN_LENGTH / 2 + 19.5, SCREEN_WIDTH / 2 - 320);
// 		// // ��תװ�װ�
// 		// Line_Draw(&UI_shoot_line[6], "sl6", UI_Graph_ADD, 7, UI_Color_White, 1, SCREEN_LENGTH / 2 + 1.5, SCREEN_WIDTH / 2 - 120, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 120);
// 		// Line_Draw(&UI_shoot_line[7], "sl7", UI_Graph_ADD, 7, UI_Color_White, 1, SCREEN_LENGTH / 2 + 19.5, SCREEN_WIDTH / 2 - 120, SCREEN_LENGTH / 2 + 31.5, SCREEN_WIDTH / 2 - 120);
// 		// // �����ͼ��
// 		// Line_Draw(&UI_shoot_line[8], "sl8", UI_Graph_ADD, 7, UI_Color_Purplish_red, 3, SCREEN_LENGTH / 2 + 7, SCREEN_WIDTH / 2 - 200, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 200);
// 		// Line_Draw(&UI_shoot_line[9], "sl9", UI_Graph_ADD, 7, UI_Color_Purplish_red, 3, SCREEN_LENGTH / 2 + 19.5, SCREEN_WIDTH / 2 - 200, SCREEN_LENGTH / 2 + 26, SCREEN_WIDTH / 2 - 200);
// 		// // ������
// 		// Line_Draw(&UI_shoot_line[10], "sa0", UI_Graph_ADD, 7, UI_Color_Orange, 2, SCREEN_LENGTH / 2 + 4, SCREEN_WIDTH / 2 - 155, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 155); // 176  280
// 		// Line_Draw(&UI_shoot_line[11], "sa1", UI_Graph_ADD, 7, UI_Color_Orange, 2, SCREEN_LENGTH / 2 + 19.5, SCREEN_WIDTH / 2 - 155, SCREEN_LENGTH / 2 + 29, SCREEN_WIDTH / 2 - 155);
// 		// // ������
// 		// Line_Draw(&UI_shoot_line[12], "sa2", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 280, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 280);
// 		// Line_Draw(&UI_shoot_line[13], "sa3", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 260, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 260);
// 		// Line_Draw(&UI_shoot_line[14], "sa4", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 240, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 240);
// 		// Line_Draw(&UI_shoot_line[15], "sa5", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 220, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 220);
// 		// Line_Draw(&UI_shoot_line[16], "sa6", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 180, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 180);
// 		// Line_Draw(&UI_shoot_line[17], "sa7", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 160, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 160);
// 		// Line_Draw(&UI_shoot_line[18], "sa8", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 140, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 140);
// 		// // �ֽ���
// 		// Line_Draw(&UI_shoot_line[19], "sa9", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 100, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 100);
// 		// Line_Draw(&UI_shoot_line[20], "sb0", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 80, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 80);
// 		// Line_Draw(&UI_shoot_line[21], "sb1", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 60, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 60);
// 		// Line_Draw(&UI_shoot_line[22], "sb2", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 40, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 40);
// 		// Line_Draw(&UI_shoot_line[23], "sb3", UI_Graph_ADD, 7, UI_Color_Black, 2, SCREEN_LENGTH / 2 + 8, SCREEN_WIDTH / 2 - 20, SCREEN_LENGTH / 2 + 13.5, SCREEN_WIDTH / 2 - 20);

// 		// 工程控制模式显示
// 		switch (Control_Mode)
// 		{
// 		case Pc_Control:
// 		{
// 			sprintf(Char_Engineer_Control_Mode, "Control:%s", "Pc_Control");
// 			Char_Draw(&UI_Engineer_Mode_String[0], "ss0", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 1, 70, 857, Char_Engineer_Control_Mode);
// 			break;
// 		}
// 		case Remote_Control:
// 		{
// 			sprintf(Char_Engineer_Control_Mode, "Control:%s", "Remote_Control");
// 			Char_Draw(&UI_Engineer_Mode_String[0], "ss0", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 1, 70, 857, Char_Engineer_Control_Mode);
// 			break;
// 		}
// 		}

// 		// 工程翻转电机状态显示
// 		switch (Air_Get_Mine_Motor)
// 		{
// 		case Air_Get_Mine_Motor_Mode_On:
// 		{
// 			sprintf(Char_Engineer_Air_Get_Mine_Motor_Mode, "Air_Motor");
// 			Char_Draw(&UI_Engineer_Mode_String[1], "ss1", UI_Graph_ADD, 9, UI_Color_Purplish_red, 15, 1, 70, 800, Char_Engineer_Air_Get_Mine_Motor_Mode);
// 			break;
// 		}
// 		case Air_Get_Mine_Motor_Mode_Off:
// 		{
// 			sprintf(Char_Engineer_Air_Get_Mine_Motor_Mode, "Air_Motor");
// 			Char_Draw(&UI_Engineer_Mode_String[1], "ss1", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 1, 70, 800, Char_Engineer_Air_Get_Mine_Motor_Mode);
// 			break;
// 		}
// 		}

// 		// 工程气泵开关状态显示
// 		switch (Air_Switch_Mode)
// 		{
// 		case Air_Switch_Mode_ON:
// 		{
// 			sprintf(Char_Engineer_Air_Switch_Mode, "Air_Switch");
// 			Char_Draw(&UI_Engineer_Mode_String[2], "ss2", UI_Graph_ADD, 9, UI_Color_Purplish_red, 15, 1, 70, 750, Char_Engineer_Air_Switch_Mode);
// 			break;
// 		}
// 		case Air_Switch_Mode_OFF:
// 		{
// 			sprintf(Char_Engineer_Air_Switch_Mode, "Air_Switch");
// 			Char_Draw(&UI_Engineer_Mode_String[2], "ss2", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 1, 70, 750, Char_Engineer_Air_Switch_Mode);
// 			break;
// 		}
// 		}

// 		// 工程障碍块模块状态显示
// 		switch (Obstacle_Block_Switch_Mode)
// 		{
// 		case Obstacle_Block_Switch_Mode_On:
// 		{
// 			sprintf(Char_Engineer_Obstacle_Block_Switch_Mode, "Obstacle_Block");
// 			Char_Draw(&UI_Engineer_Mode_String[3], "ss3", UI_Graph_ADD, 9, UI_Color_Purplish_red, 15, 1, 70, 700, Char_Engineer_Obstacle_Block_Switch_Mode);
// 			break;
// 		}
// 		case Obstacle_Block_Switch_Mode_Off:
// 		{
// 			sprintf(Char_Engineer_Obstacle_Block_Switch_Mode, "Obstacle_Block");
// 			Char_Draw(&UI_Engineer_Mode_String[3], "ss3", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 1, 70, 700, Char_Engineer_Obstacle_Block_Switch_Mode);
// 			break;
// 		}
// 		}

// 		// 工程涵道模块状态显示
// 		switch (Duct_Switch_Mode)
// 		{
// 		case Duct_Switch_Mode_On:
// 		{
// 			sprintf(Char_Engineer_Duct_Switch_Mode, "Duct_Switch");
// 			Char_Draw(&UI_Engineer_Mode_String[4], "ss4", UI_Graph_ADD, 9, UI_Color_Purplish_red, 15, 1, 1639, 857, Char_Engineer_Duct_Switch_Mode);
// 			break;
// 		}
// 		case Obstacle_Block_Switch_Mode_Off:
// 		{
// 			sprintf(Char_Engineer_Duct_Switch_Mode, "Duct_Switch");
// 			Char_Draw(&UI_Engineer_Mode_String[4], "ss4", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 1, 1639, 857, Char_Engineer_Duct_Switch_Mode);
// 			break;
// 		}
// 		}

// 		// 工程矿仓拓展模块状态显示
// 		switch (Mine_Warehouse_Expand_Switch_Mode)
// 		{
// 		case Mine_Warehouse_Expand_Switch_Mode_On:
// 		{
// 			sprintf(Char_Engineer_Mine_Warehouse_Expand_Switch_Mode, "Mine_Expand");
// 			Char_Draw(&UI_Engineer_Mode_String[5], "ss5", UI_Graph_ADD, 9, UI_Color_Purplish_red, 15, 1, 1639, 800, Char_Engineer_Mine_Warehouse_Expand_Switch_Mode);
// 			break;
// 		}
// 		case Mine_Warehouse_Expand_Switch_Mode_Off:
// 		{
// 			sprintf(Char_Engineer_Mine_Warehouse_Expand_Switch_Mode, "Mine_Expand");
// 			Char_Draw(&UI_Engineer_Mode_String[5], "ss5", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 1, 1639, 800, Char_Engineer_Mine_Warehouse_Expand_Switch_Mode);
// 			break;
// 		}
// 		}

// 		// 工程矿仓固定模块状态显示
// 		switch (Mine_Warehouse_Lock_Switch_Mode)
// 		{
// 		case Mine_Warehouse_Lock_Switch_Mode_On:
// 		{
// 			sprintf(Char_Engineer_Mine_Warehouse_Expand_Lock_Mode, "Mine_Lock");
// 			Char_Draw(&UI_Engineer_Mode_String[6], "ss6", UI_Graph_ADD, 9, UI_Color_Purplish_red, 15, 1, 1639, 750, Char_Engineer_Mine_Warehouse_Expand_Lock_Mode);
// 			break;
// 		}
// 		case Mine_Warehouse_Lock_Switch_Mode_Off:
// 		{
// 			sprintf(Char_Engineer_Mine_Warehouse_Expand_Lock_Mode, "Mine_Lock");
// 			Char_Draw(&UI_Engineer_Mode_String[6], "ss6", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 1, 1639, 750, Char_Engineer_Mine_Warehouse_Expand_Lock_Mode);
// 			break;
// 		}
// 		}

// 		// 工程空接吸盘模块状态显示
// 		switch (Air_Switch_Dirction_Mode)
// 		{
// 		case Air_Switch_Dirction_Mode_Up:
// 		{
// 			sprintf(Char_Engineer_Air_Switch_Dirction_Mode, "Air_Switch_Dirction");
// 			Char_Draw(&UI_Engineer_Mode_String[7], "ss7", UI_Graph_ADD, 9, UI_Color_Purplish_red, 15, 1, 1639, 700, Char_Engineer_Air_Switch_Dirction_Mode);
// 			break;
// 		}
// 		case Air_Switch_Dirction_Mode_Middle:
// 		{
// 			sprintf(Char_Engineer_Air_Switch_Dirction_Mode, "Air_Switch_Dirction");
// 			Char_Draw(&UI_Engineer_Mode_String[7], "ss7", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 1, 1639, 700, Char_Engineer_Air_Switch_Dirction_Mode);
// 			break;
// 		}
// 		}

// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[0]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[1]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[2]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[3]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[4]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[5]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[6]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[7]);

// 		UI_ReFresh(&referee_recv_info->referee_id, 4, UI_Enginer_Air_Mine_Line[0], UI_Enginer_Air_Mine_Line[1], UI_Enginer_Obstacle_Block_Line[0], UI_Enginer_Obstacle_Block_Line[1]);

// 		Send_Once_Flag = 1;
// 	}

// 	else
// 	{

// 		// 工程控制模式显示
// 		switch (Control_Mode)
// 		{
// 		case Pc_Control:
// 		{
// 			sprintf(Char_Engineer_Control_Mode, "Control:%s", "Pc_Control");
// 			Char_Draw(&UI_Engineer_Mode_String[0], "ss0", UI_Graph_Change, 9, UI_Color_Yellow, 15, 1, 70, 857, Char_Engineer_Control_Mode);
// 			break;
// 		}
// 		case Remote_Control:
// 		{
// 			sprintf(Char_Engineer_Control_Mode, "Control:%s", "Remote_Control");
// 			Char_Draw(&UI_Engineer_Mode_String[0], "ss0", UI_Graph_Change, 9, UI_Color_Yellow, 15, 1, 70, 857, Char_Engineer_Control_Mode);
// 			break;
// 		}
// 		}

// 		// 工程翻转电机状态显示
// 		switch (Air_Get_Mine_Motor)
// 		{
// 		case Air_Get_Mine_Motor_Mode_On:
// 		{
// 			sprintf(Char_Engineer_Air_Get_Mine_Motor_Mode, "Air_Motor");
// 			Char_Draw(&UI_Engineer_Mode_String[1], "ss1", UI_Graph_Change, 9, UI_Color_Purplish_red, 15, 1, 70, 800, Char_Engineer_Air_Get_Mine_Motor_Mode);
// 			break;
// 		}
// 		case Air_Get_Mine_Motor_Mode_Off:
// 		{
// 			sprintf(Char_Engineer_Air_Get_Mine_Motor_Mode, "Air_Motor");
// 			Char_Draw(&UI_Engineer_Mode_String[1], "ss1", UI_Graph_Change, 9, UI_Color_Yellow, 15, 1, 70, 800, Char_Engineer_Air_Get_Mine_Motor_Mode);
// 			break;
// 		}
// 		}

// 		// 工程气泵开关状态显示
// 		switch (Air_Switch_Mode)
// 		{
// 		case Air_Switch_Mode_ON:
// 		{
// 			sprintf(Char_Engineer_Air_Switch_Mode, "Air_Switch");
// 			Char_Draw(&UI_Engineer_Mode_String[2], "ss2", UI_Graph_Change, 9, UI_Color_Purplish_red, 15, 1, 70, 750, Char_Engineer_Air_Switch_Mode);
// 			break;
// 		}
// 		case Air_Switch_Mode_OFF:
// 		{
// 			sprintf(Char_Engineer_Air_Switch_Mode, "Air_Switch");
// 			Char_Draw(&UI_Engineer_Mode_String[2], "ss2", UI_Graph_Change, 9, UI_Color_Yellow, 15, 1, 70, 750, Char_Engineer_Air_Switch_Mode);
// 			break;
// 		}
// 		}

// 		// 工程障碍块模块状态显示
// 		switch (Obstacle_Block_Switch_Mode)
// 		{
// 		case Obstacle_Block_Switch_Mode_On:
// 		{
// 			sprintf(Char_Engineer_Obstacle_Block_Switch_Mode, "Obstacle_Block");
// 			Char_Draw(&UI_Engineer_Mode_String[3], "ss3", UI_Graph_Change, 9, UI_Color_Purplish_red, 15, 1, 70, 700, Char_Engineer_Obstacle_Block_Switch_Mode);
// 			break;
// 		}
// 		case Obstacle_Block_Switch_Mode_Off:
// 		{
// 			sprintf(Char_Engineer_Obstacle_Block_Switch_Mode, "Obstacle_Block");
// 			Char_Draw(&UI_Engineer_Mode_String[3], "ss3", UI_Graph_Change, 9, UI_Color_Yellow, 15, 1, 70, 700, Char_Engineer_Obstacle_Block_Switch_Mode);
// 			break;
// 		}
// 		}

// 		// 工程涵道模块状态显示
// 		switch (Duct_Switch_Mode)
// 		{
// 		case Duct_Switch_Mode_On:
// 		{
// 			sprintf(Char_Engineer_Duct_Switch_Mode, "Duct_Switch");
// 			Char_Draw(&UI_Engineer_Mode_String[4], "ss4", UI_Graph_Change, 9, UI_Color_Purplish_red, 15, 1, 1639, 857, Char_Engineer_Duct_Switch_Mode);
// 			break;
// 		}
// 		case Obstacle_Block_Switch_Mode_Off:
// 		{
// 			sprintf(Char_Engineer_Duct_Switch_Mode, "Duct_Switch");
// 			Char_Draw(&UI_Engineer_Mode_String[4], "ss4", UI_Graph_Change, 9, UI_Color_Yellow, 15, 1, 1639, 857, Char_Engineer_Duct_Switch_Mode);
// 			break;
// 		}
// 		}

// 		// 工程矿仓拓展模块状态显示
// 		switch (Mine_Warehouse_Expand_Switch_Mode)
// 		{
// 		case Mine_Warehouse_Expand_Switch_Mode_On:
// 		{
// 			sprintf(Char_Engineer_Mine_Warehouse_Expand_Switch_Mode, "Mine_Expand");
// 			Char_Draw(&UI_Engineer_Mode_String[5], "ss5", UI_Graph_Change, 9, UI_Color_Purplish_red, 15, 1, 1639, 800, Char_Engineer_Mine_Warehouse_Expand_Switch_Mode);
// 			break;
// 		}
// 		case Mine_Warehouse_Expand_Switch_Mode_Off:
// 		{
// 			sprintf(Char_Engineer_Mine_Warehouse_Expand_Switch_Mode, "Mine_Expand");
// 			Char_Draw(&UI_Engineer_Mode_String[5], "ss5", UI_Graph_Change, 9, UI_Color_Yellow, 15, 1, 1639, 800, Char_Engineer_Mine_Warehouse_Expand_Switch_Mode);
// 			break;
// 		}
// 		}

// 		// 工程矿仓固定模块状态显示
// 		switch (Mine_Warehouse_Lock_Switch_Mode)
// 		{
// 		case Mine_Warehouse_Lock_Switch_Mode_On:
// 		{
// 			sprintf(Char_Engineer_Mine_Warehouse_Expand_Lock_Mode, "Mine_Lock");
// 			Char_Draw(&UI_Engineer_Mode_String[6], "ss6", UI_Graph_Change, 9, UI_Color_Purplish_red, 15, 1, 1639, 750, Char_Engineer_Mine_Warehouse_Expand_Lock_Mode);
// 			break;
// 		}
// 		case Mine_Warehouse_Lock_Switch_Mode_Off:
// 		{
// 			sprintf(Char_Engineer_Mine_Warehouse_Expand_Lock_Mode, "Mine_Lock");
// 			Char_Draw(&UI_Engineer_Mode_String[6], "ss6", UI_Graph_Change, 9, UI_Color_Yellow, 15, 1, 1639, 750, Char_Engineer_Mine_Warehouse_Expand_Lock_Mode);
// 			break;
// 		}
// 		}

// 		// 工程空接吸盘模块状态显示
// 		switch (Air_Switch_Dirction_Mode)
// 		{
// 		case Air_Switch_Dirction_Mode_Up:
// 		{
// 			sprintf(Char_Engineer_Air_Switch_Dirction_Mode, "Air_Switch_Dirction");
// 			Char_Draw(&UI_Engineer_Mode_String[7], "ss7", UI_Graph_Change, 9, UI_Color_Purplish_red, 15, 1, 1639, 700, Char_Engineer_Air_Switch_Dirction_Mode);
// 			break;
// 		}
// 		case Air_Switch_Dirction_Mode_Middle:
// 		{
// 			sprintf(Char_Engineer_Air_Switch_Dirction_Mode, "Air_Switch_Dirction");
// 			Char_Draw(&UI_Engineer_Mode_String[7], "ss7", UI_Graph_Change, 9, UI_Color_Yellow, 15, 1, 1639, 700, Char_Engineer_Air_Switch_Dirction_Mode);
// 			break;
// 		}
// 		}

// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[0]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[1]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[2]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[3]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[4]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[5]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[6]);
// 		Char_ReFresh(&referee_recv_info->referee_id, UI_Engineer_Mode_String[7]);
// 	}
// }

// /**
//  * @brief ����ϵͳ���ݷ��ͺ���
//  * @param
//  */
// void RefereeSend(uint8_t *send, uint16_t tx_len)
// {
// 	static TickType_t xLastWakeTime;
// 	while (HAL_DMA_GetState(huart6.hdmatx) != HAL_DMA_STATE_READY)
// 		;
// 	__HAL_DMA_DISABLE(huart6.hdmatx);
// 	HAL_UART_Transmit_DMA(&huart6, send, tx_len);
// 	vTaskDelayUntil(&xLastWakeTime, 120);
// }
