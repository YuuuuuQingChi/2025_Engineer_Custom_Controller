/**
 * @Author: HDC h2019dc@outlook.com
 * @Date: 2023-09-08 16:47:43
 * @LastEditors: HDC h2019dc@outlook.com
 * @LastEditTime: 2023-10-26 21:51:44
 * @FilePath: \2024_Control_New_Framework_Base-dev-all\application\robot.c
 * @Description:
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "buzzer.h"
#include "chassis.h"
#include "lift.h"
#include "arm.h"
#include "robot_cmd.h"
#include "stretch.h"
#include "referee_UI.h"
#include "ui.h"
#include "servo.h"
#include "vision_line.h"

//int32_t flag_referee_init =0;
//static referee_info_t *referee_data; // 用于获取裁判系统的数据

void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();

    BSPInit();
    RobotCMDInit();
    ChassisInit();
    Servo_Init();
    //referee_data = RefereeHardwareInit(&huart10); // 裁判系统初始化,会同时初始化UI
    //flag_referee_init=1;                 
    Lift_Init();
    Stretch_Init();
    ARM_Init();
    Vision_Int();
    //uiInit();
    Vision_Int();
    __enable_irq();
}

void RobotTask()
{
    RobotCMDTask();
    ChassisTask();
    Lift_Task();
    Stretch_Task();
    Servo_Task();
    ARM_Task();
    Vision_Task();
}