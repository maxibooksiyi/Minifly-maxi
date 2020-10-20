#include <math.h>
#include "flip.h"
#include "config_param.h"
#include "commander.h"
#include "stabilizer.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 四轴空翻控制代码	
 
 
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define FLIP_RATE		RATE_500_HZ				/*周期*/	
#define MID_ANGLE		(180.f * FLIP_RATE)		/*中间角度 放大500倍*/
#define MAX_FLIP_RATE	1380					/* <2000 */
#define DELTA_RATE		(30000.f/MAX_FLIP_RATE)	/*递增速率*/

#define FLIP_TIMEOUT		800			/*翻滚过程超时时间*/
#define SPEED_UP_TIMEOUT	400			/*加速上升超时时间*/
#define REVER_SPEEDUP_TIME	160			/*反向加速时间*/


static enum
{
	FLIP_IDLE = 0,
	FLIP_SET,
	FLIP_SPEED_UP,
	FLIP_SLOW_DOWN,
	FLIP_PERIOD,
	FLIP_FINISHED,
	REVER_SPEED_UP,
	FLIP_ERROR,
}flipState = FLIP_IDLE;

u8 fstate;				/*翻滚状态*/
enum dir_e flipDir;		/*翻滚方向*/
static u16 maxRateCnt = 0;			/*最大速率计数*/
static float desiredVelZ = 105.f;	/*加速上升期望速度*/
static float currentRate = 0.f;		/*当前速率*/
static float currentAngle = 0.f;	/*当前角度 放大500倍*/
bool isExitFlip = true;				/*是否退出空翻*/

/********************************************************
* Flyer 翻滚检测 
*********************************************************/
void flyerFlipCheck(setpoint_t* setpoint, control_t* control, state_t* state)  //maxi:注意这几个传入的结构体。这个没弄清楚，后面就会看得比较乱。
{
	static u16 flipThrust = 0;
	static u16 tempThrust = 0;
	static u16 reverTime = 0;
	static u16 flipTimeout = 0;	
	static float pitchTemp = 0.0;
	static float rollTemp = 0.0;
	static float yawTemp = 0.0;
	static float deltaThrust = 0.0;
	static u16 exitFlipCnt = 0;
	static u16 flipThrustMax = 0;

	fstate = (u8)flipState;
	
	switch(flipState)
	{
		case FLIP_IDLE:	/*翻滚空闲状态*/
		{
			if(flipDir!=CENTER)
			{
				if(control->thrust > 28000 && state->velocity.z > -20.f)
				{
					flipState = FLIP_SET;
					exitFlipCnt = 500;		/*空翻完成，延时1S(500Hz)退出空翻 */
					isExitFlip = false;
				}					
				else
				{
					flipDir = CENTER;					
				}					
			}else if(isExitFlip == false)
			{
				if(exitFlipCnt > 0) 
					exitFlipCnt--;
				else
					isExitFlip = true;
			}				
			break;
		}
		case FLIP_SET:	/*翻滚设置*/
		{
			currentRate = 0.f;
			maxRateCnt = 0;
			currentAngle = 0.f;
			
			flipTimeout = 0;
			control->flipDir = flipDir;
			flipThrust = -9000.0f + 1.2f * configParam.thrustBase;
			deltaThrust = configParam.thrustBase / 90.0f;
			flipThrustMax = configParam.thrustBase + 20000;
			if(flipThrustMax > 62000) flipThrustMax = 62000;
			tempThrust = flipThrust; 
					
			rollTemp = state->attitude.roll;
			pitchTemp = state->attitude.pitch;									
			yawTemp = state->attitude.yaw;
			
			flipState = FLIP_SPEED_UP;
			break;
		}
		case FLIP_SPEED_UP:	/*加速上升过程*/
		{
			if(state->velocity.z < desiredVelZ)
			{
				setpoint->mode.z = modeDisable;
				if(tempThrust < flipThrustMax)
					tempThrust += deltaThrust;
				setpoint->thrust = tempThrust;
				
				if(flipTimeout++ > SPEED_UP_TIMEOUT)	/*超时处理*/
				{
					flipTimeout = 0;
					flipState = FLIP_SLOW_DOWN;			/*直接进入下一个状态*/
				}														
			}else	
			{	
				flipTimeout = 0;				
				flipState = FLIP_SLOW_DOWN;
			}		
			break;
		}
		case FLIP_SLOW_DOWN:	/*减速过程*/
		{
			if(tempThrust > flipThrust)
			{
				tempThrust -= (6500.f - flipThrust / 10.0f);
				setpoint->mode.z = modeDisable;
				setpoint->thrust = tempThrust;
			}else
			{
				flipState = FLIP_PERIOD;
			}
		}
		case FLIP_PERIOD:	/*翻滚过程*/
		{
			if(flipTimeout++ > FLIP_TIMEOUT)	/*超时处理*/
			{
				flipTimeout = 0;
				flipState = FLIP_ERROR;
			}
			
			setpoint->mode.z = modeDisable;
			setpoint->thrust = flipThrust - 3*currentRate;
			
			currentAngle += currentRate;		/*当前角度 放大500倍*/
			                               //maxi:当前角度是当前速率累加起来的,
			                               //maxi:(这是开发指南里面的话）FLIP_PERIOD 状态下，我们按照设定方向设置四轴角速度的值，由于只是用了内环， 没使用外环角度环，四轴就没有自稳模式，四轴就会按照设定方向以设定角速度转动，同时 我们对角速度积分，当这个积分值达到 360°，说明四轴绕某个轴转动一圈了，然后状态切 换到 FLIP_FINISHED。这就是实际空翻过程。 
			if(currentAngle < MID_ANGLE)		/*上半圈*/
			{
				if(currentRate < MAX_FLIP_RATE)/*小于最大速率，速率继续增大*/
					currentRate += DELTA_RATE;               //maxi:DELTA_RATE是递增速率，感觉翻滚没有用PID(用了），cureenrRate应该控制着电机的输出，看189行，赋值给。还有我感觉它的期望角速度是渐变的，逐渐变到最大值然后保持不变，而不是突变，一下子把角速度期望值变为某一个值，可能这样是为了无人机姿态稳定些？
				else			/*大于最大速率，速率保持*/
					maxRateCnt++;					//maxi:maxRateCnt的值原本是0，这个的值好像要么是0要么是1，用来判断是否达到最大速率。
			}else	/*下半圈*/
			{
				if(maxRateCnt > 0)
				{						
					maxRateCnt--;			
				}else
				{
					if(currentRate >= DELTA_RATE && currentAngle < 2*MID_ANGLE)
					{
						currentRate -= DELTA_RATE;	  //maxi:下半圈就让叫嘟嘟降低
					}																
					else							
						flipState = FLIP_FINISHED;						
				}
			}
			
			switch(control->flipDir)	   //maxi:这是选择翻滚的方向，前后左右，影响的应该是翻滚角或俯仰角
			{
				case FORWARD:	/* pitch+ */
					setpoint->attitude.pitch = currentRate;	         //maxi:currentRate应该是控制着相应的电机的量的，这个赋值应该是这个意思。或者说是期望的角速度？
				  setpoint->attitude.roll = state->attitude.roll = rollTemp;   //maxi:setpoint_t* setpoint是传入函数的一个结构体，包含角度，角速度，速度，位置，模式，油门。
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;				
				case BACK:		/* pitch- */
					setpoint->attitude.pitch = -currentRate;
					setpoint->attitude.roll = state->attitude.roll = rollTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;
				case LEFT:		/* roll- */
					setpoint->attitude.roll = -currentRate;	
					setpoint->attitude.pitch = state->attitude.pitch = pitchTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;
				case RIGHT:		/* roll+ */					
					setpoint->attitude.roll = currentRate;
					setpoint->attitude.pitch = state->attitude.pitch = pitchTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;
				default :break;
			}
			break;
		}
		case FLIP_FINISHED:	/*翻滚完成*/
		{
			setpoint->mode.z = modeDisable;
			setpoint->thrust = tempThrust;
			tempThrust = flipThrust;
			
			reverTime = 0;
			flipTimeout = 0;
			flipDir = CENTER;	
			control->flipDir = flipDir;

			flipState = REVER_SPEED_UP;
			break;
		}
		case REVER_SPEED_UP:	/*翻滚完成后 反向加速*/
		{			
			if(reverTime++<REVER_SPEEDUP_TIME)	
			{
				if(tempThrust < flipThrustMax)
					tempThrust += 2.0f * deltaThrust;
				setpoint->mode.z = modeDisable;
				setpoint->thrust = tempThrust;
			}else
			{				
				flipTimeout = 0;
				flipState = FLIP_IDLE;					
//				if(getCommanderKeyFlight())	/*定高模式*/
//				{
//					setpoint->thrust = 0;
//					setpoint->mode.z = modeAbs;	
//				}
			}
			break;
		}
		case FLIP_ERROR:
		{
			reverTime = 0;
			flipDir = CENTER;	
			control->flipDir = CENTER;
			
			setpoint->mode.z = modeDisable;
			setpoint->thrust = 0;
			if(flipTimeout++ > 1)
			{
				flipTimeout = 0;
				
				if(getCommanderKeyFlight())	/*定高模式*/
				{
					setpoint->thrust = 0;
					setpoint->mode.z = modeAbs;				
				}
				
				flipState = FLIP_IDLE;
			}
			break;
		}
		default : break;
	}			
}


//设置翻滚方向
void setFlipDir(u8 dir)
{
	flipDir = (enum dir_e)dir;	
}

