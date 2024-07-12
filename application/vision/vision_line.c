#include "robot_def.h"
#include "robot_cmd.h"

// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "can_comm.h"
#include "buzzer.h"
#include "led.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "referee_protocol.h"
#include "rm_referee.h"
#include "vision_line.h"
#include "crc_ref.h"

#define stretch_scale 3.502042
// 图传链路
extern USARTInstance *vision_usart;

static Publisher_t *vision_joint_data_pub;
static Subscriber_t *vision_joint_data_sub;
static Vision_Joint_Data_Upload_Data_s Vision_Joint_Data_feedback_data; 
static Vision_Joint_Data_Ctrl_Cmd_s Vision_Joint_Data_cmd_recv;  

static referee_info_t referee_vision_info;			  // 图传链路数据
static uint8_t vision_recv_data[30];
static uint8_t custom_controller_comm_recv; //自定义发送的自定义标志位	
static uint8_t run_flag;
static uint8_t roll_data;
static uint8_t lift_data;
static uint8_t air_pump;
static float stretch;
static float encoder_Data[4];
//分析图传数据
void JudgeVisionReadData(uint8_t* buff){
   // uint16_t judge_length; // 统计一帧数据长度
	// if (buff == NULL)	   // 空数据包，则不作任何处理
	// 	return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_vision_info.FrameHeader, buff, LEN_HEADER);

	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		
			if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
	//		judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID ;
			
				// 2个8位拼成16位int
				referee_vision_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
			
				switch (referee_vision_info.CmdID)
				{
                    case 0x0302: //自定义控制器
                        memcpy(&vision_recv_data, (buff + DATA_Offset), 30);//一起写入

                        memcpy((uint8_t*)&custom_controller_comm_recv,(uint8_t*)vision_recv_data,1);//对应的0xff
                        memcpy((uint8_t*)&run_flag,(uint8_t*)vision_recv_data+1,1);//启动！！！
                        memcpy((uint8_t*)encoder_Data,(uint8_t*)vision_recv_data+2,16);//四个关节
						memcpy((uint8_t*)&roll_data,(uint8_t*)vision_recv_data+18,1);//roll的启停
						memcpy((uint8_t*)&stretch,(uint8_t*)vision_recv_data+19,4);//伸出的角度
						memcpy((uint8_t*)&lift_data,(uint8_t*)vision_recv_data+23,1);//升降的标志
						memcpy((uint8_t*)&air_pump,(uint8_t*)vision_recv_data+24,1);//气泵的的标志

					    break;
					
                    default:
                        break;
                }
			
		}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_vision_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			JudgeVisionReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_vision_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}


void vision_recv_callback(){
    JudgeVisionReadData(vision_usart->recv_buff);
}

//只有消息中心的初始化目前是这样的
void Vision_Int()
{
	vision_joint_data_pub = PubRegister("vision_joint_data_feed", sizeof(Vision_Joint_Data_Upload_Data_s));
    vision_joint_data_sub = SubRegister("vision_joint_data_cmd", sizeof(Vision_Joint_Data_Ctrl_Cmd_s));

}

//只做消息的转发
void Vision_Task()
{
	SubGetMessage(vision_joint_data_sub, &Vision_Joint_Data_cmd_recv);

	Vision_Joint_Data_feedback_data.vision_big_pitch = - encoder_Data[3]/ 180.0 * PI;
	Vision_Joint_Data_feedback_data.vision_big_roll = encoder_Data[0] / 180.0 * PI;
	Vision_Joint_Data_feedback_data.vision_small_pitch = encoder_Data[1]/ 180.0 * PI;
	Vision_Joint_Data_feedback_data.vision_small_roll = (roll_data == 2 ? 8000 : (roll_data == 3 ? -8000 : 0));
	Vision_Joint_Data_feedback_data.vision_yaw = encoder_Data[2]/ 180.0 * PI;
	Vision_Joint_Data_feedback_data.custom_controller_comm_recv = custom_controller_comm_recv;
	Vision_Joint_Data_feedback_data.run_flag = run_flag;
	Vision_Joint_Data_feedback_data.vision_stretch = (stretch - 205.311035)/ stretch_scale;
	Vision_Joint_Data_feedback_data.lift = lift_data;
	Vision_Joint_Data_feedback_data.air_pump = air_pump;

	PubPushMessage(vision_joint_data_pub, (void *)&Vision_Joint_Data_feedback_data);

}

