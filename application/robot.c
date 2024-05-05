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
#include "first.h"
#include "robot_cmd.h"
#include "second.h"
#include "horizontal.h"
#include "forward.h"
#include "referee_UI.h"

void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();

    BSPInit();
  // buzzer_one_note(Do_freq, 0.1);
    RobotCMDInit();
  //BuzzerPlay(RoboMaster_You);
   // buzzer_one_note(La_freq, 1);
 ChassisInit();                  
   // buzzer_one_note(So_freq, 0.1f);
    Lift_Init();
    First_Stretch_Init();
    Second_Stretch_Init();
      //  buzzer_one_note(Mi_freq, 1);

  Horizontal_Init();
    Forward_Init();
    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
    RobotCMDTask();
    ChassisTask();
    Lift_Task();
   First_Stretch_Task();
    Second_Stretch_Task();
    Horizontal_Task();
    Forward_Task();
    //My_UIGraphRefresh();
}