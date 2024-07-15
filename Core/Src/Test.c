/*
 * Test.c
 *
 *  Created on: Apr 21, 2024
 *      Author: wsrra
 */
#include "Test.h"
#include "OD_PWM.h"
#include "main.h"
#include "debug.h"
#include "math.h"

extern GVar_ts gVar;
extern TIM_HandleTypeDef htim14;


void Test_Init(void){


	gVar.triac_1.mode						= MODULATION_TYPE;
	gVar.triac_2.mode						= MODULATION_TYPE;


	gVar.triac_1.enabled					= 1;//0U;
	gVar.triac_1.channel_1_port 			= TRIAC_1_GPIO_Port;
	gVar.triac_1.channel_1 					= TRIAC_1_Pin;
	gVar.triac_1.counter1					= 0U;
	gVar.triac_1.counter2					= 0U;
	gVar.triac_1.dutyCycle					= DUTYCYCLE;
	gVar.triac_1.dutyCycleVar				= 1U;//0U;
	gVar.triac_1.period						= 0U;
	gVar.triac_1.incrementBy				= 0U;
	gVar.triac_1.offDuration				= 0U;
	gVar.triac_1.onDuration					= 0U;
	gVar.triac_1.isZeroCrossed				= 0U;
	gVar.triac_1.zcDelayAdjust				= CNFG_DEF_ZC_DELAY_ADJUST;			// zero cross delay adjustment for 110V 60Hz system
	gVar.triac_1.excepStatus				= 0U;



	gVar.triac_1.totalNumberOfCycle			= TOTAL_NUMBER_OF_CYCLE;
	gVar.triac_1.numberOfActiveCycle		= NUMBER_OF_ACTIVE_CYCLE;
	gVar.triac_1.numberOfInactiveCycle		= 0U;
	gVar.triac_1.nextInactiveCyclePos		= 0U;
	gVar.triac_1.inactiveCycleFactor		= 0U;
	gVar.triac_1.zeroCrossCounter					= 0U;
	gVar.triac_1.currentCyclePos			= 0U;
	gVar.triac_1.nextInactiveCycleFlag  	= 0U;


//	gVar.triac_2.frequency					= 50U;//0U;
	gVar.triac_2.enabled					= 1U; // 0U;
	gVar.triac_2.channel_1_port 			= TRIAC_2_GPIO_Port;
	gVar.triac_2.channel_1 					= TRIAC_2_Pin;
	gVar.triac_2.counter1					= 0U;
	gVar.triac_2.counter2					= 0U;
	gVar.triac_2.dutyCycle					= DUTYCYCLE;
	gVar.triac_2.dutyCycleVar				= 1U;//0U;
	gVar.triac_2.period						= 0U;
	gVar.triac_2.incrementBy				= 0U;
	gVar.triac_2.offDuration				= 0U;
	gVar.triac_2.onDuration					= 0U;
	gVar.triac_2.isZeroCrossed				= 0U;
	gVar.triac_2.zcDelayAdjust				= CNFG_DEF_ZC_DELAY_ADJUST;			// zero cross delay adjustment for 110V 60Hz system
	gVar.triac_2.excepStatus				= 0U;



	gVar.triac_2.totalNumberOfCycle			= TOTAL_NUMBER_OF_CYCLE;
	gVar.triac_2.numberOfActiveCycle		= NUMBER_OF_ACTIVE_CYCLE;
	gVar.triac_2.numberOfInactiveCycle		= 0U;
	gVar.triac_2.nextInactiveCyclePos		= 0U;
	gVar.triac_2.inactiveCycleFactor		= 0U;
	gVar.triac_2.zeroCrossCounter					= 0U;
	gVar.triac_2.currentCyclePos			= 0U;
	gVar.triac_2.nextInactiveCycleFlag  	= 0U;


	DebugPrintf("tric FM init....\r\n");




	OD_PWM_InitTriac(&htim14, &gVar.triac_1, AC_SIGNAL_FREQ);
	OD_PWM_InitTriac(&htim14, &gVar.triac_2, AC_SIGNAL_FREQ);

	DebugPrintf("%f \n", gVar.triac_1.frequency);

	HAL_TIM_Base_Start_IT(&htim14);


}


void Test_Handler(void){
	  static uint32_t tick = 0;
	  if((HAL_GetTick() - tick)>=20){

		  OD_PWM_CalcTriacPWM(&gVar.triac_1);
		  OD_PWM_CalcTriacPWM(&gVar.triac_2);
		  OD_PWM_RegulateTriacPWM(&gVar.triac_1);
		  OD_PWM_RegulateTriacPWM(&gVar.triac_2);

	  }
}

// uint32_t time_start, time_end, time_delta;
// 	double value = 3.14159;
// 	int rounded =0 ;

// void  ExecutionTimeCalculate()
// {

//	HAL_TIM_Base_Start(&htim14);
//	time_start = __HAL_TIM_GET_COUNTER(&htim14);
//	rounded  = round(value);
//	time_end = __HAL_TIM_GET_COUNTER(&htim14);
//	time_delta = time_end - time_start;
//	DebugPrintf("time : %lu, %lu , %lu\n", time_start, time_end,time_delta);
//	HAL_TIM_Base_Stop(&htim14);
//
//
//	HAL_TIM_Base_Start(&htim14);
//	time_start = __HAL_TIM_GET_COUNTER(&htim14);
//	HAL_Delay(1500);
//	time_end = __HAL_TIM_GET_COUNTER(&htim14);
//	time_delta = time_end - time_start;
//	DebugPrintf("time : %lu, %lu , %lu\n", time_start, time_end,time_delta);
//	HAL_TIM_Base_Stop(&htim14);
//
//
//	HAL_TIM_Base_Start(&htim14);
//	time_start = __HAL_TIM_GET_COUNTER(&htim14);
//	HAL_Delay(1);
//	time_end = __HAL_TIM_GET_COUNTER(&htim14);
//	time_delta = time_end - time_start;
//	DebugPrintf("time : %lu, %lu , %lu\n", time_start, time_end,time_delta);
//	HAL_TIM_Base_Stop(&htim14);
//
//
//	HAL_TIM_Base_Start(&htim14);
//	time_start = __HAL_TIM_GET_COUNTER(&htim14);
//	rounded  = round(value);
//	time_end = __HAL_TIM_GET_COUNTER(&htim14);
//	time_delta = time_end - time_start;
//	DebugPrintf("time : %lu, %lu , %lu\n", time_start, time_end,time_delta);
//	HAL_TIM_Base_Stop(&htim14);

// }
