/*
 * OD_PWM.c
 *
 *  Created on: Apr 16, 2022
 *      Author: RASEL_EEE
 */

#include "OD_PWM.h"
#include "Test.h"
//#include "OD_Defines.h"
#include "math.h"
#include "debug.h"
//#include "Debug.h"
//#include "OD_Settings.h"


uint16_t increment_by = 0U;

//Driver board index

/*------------------ Triac 1 & 2 --------------------- */

/**
 * @brief initialize triac 1&2
 * @param htim: timer for the triac pwm
 * @param triacPwm: triac related struct
 * triacPwm acLineFrequency: AC line frequency, Asia- 50Hz, EU-60Hz
 * @retval none
 */
void OD_PWM_InitTriac(TIM_HandleTypeDef *htim, PWM *triacPwm,
		uint16_t acLineFrequency) {
	
	if(triacPwm->mode == PHASE_MODULATION)
	{
		OD_PWM_InitTriac_PM(htim, triacPwm,acLineFrequency);
	}
	else if (triacPwm->mode == FREQUENCY_MODULATION)
	{
		OD_PWM_InitTriac_FM(htim, triacPwm,acLineFrequency);
	}
}

void OD_PWM_InitTriac_PM(TIM_HandleTypeDef *htim, PWM *triacPwm,
		uint16_t acLineFrequency) {
	//HAL_TIM_Base_Start_IT(htim);
	triacPwm->incrementBy = htim->Init.Period;
	triacPwm->pulse = (triacPwm->incrementBy
			* OD_PWM_TRIAC_TRIG_IMPULSE_DURATION);
	/*---------- PWM For Triac 1 & 2-----------*/
	triacPwm->frequency = (acLineFrequency * 2U);
	triacPwm->counter1 = 0U;
	triacPwm->counter2 = 0U;

	DebugPrintf("init triac frq = %2f \n ", triacPwm->frequency);
	DebugPrintf("init acline frq = %d\n",acLineFrequency);


}


void OD_PWM_InitTriac_FM(TIM_HandleTypeDef *htim, PWM *triacPwm, uint16_t acLineFrequency)
{
	triacPwm->frequency = acLineFrequency;
	if(triacPwm->frequency > 0 && triacPwm->numberOfActiveCycle < TOTAL_NUMBER_OF_CYCLE)
	{
		triacPwm->numberOfInactiveCycle = triacPwm->totalNumberOfCycle - triacPwm->numberOfActiveCycle;
		triacPwm->inactiveCycleFactor = triacPwm-> totalNumberOfCycle * 1.0/ triacPwm->numberOfInactiveCycle;

		triacPwm->nextInactiveCycleFrac = 0;
		triacPwm->nextInactiveCyclePos = 0;
		DebugPrintf("factor %f - inact pos %d\r\n", triacPwm->inactiveCycleFactor,triacPwm->nextInactiveCyclePos );
	}
}

/**
 * @brief calculate pwm for triac 1&2
 * #For 50 System
 * there is delay between ac signal zero cross and zero cross detection by 1500us
 * thats why, to eliminate this issue we will take action after 1 1/2cycle,
 * so we have to shift about 10000-1500 = 8500 us for the next 1/2cycle
 * #For 60 System
 * there is delay between ac signal zero cross and zero cross detection by 1500us
 * thats why, to eliminate this issue we will take action after 1 1/2cycle,
 * so we have to shift about 8333-1500 = 6833 us for the next 1/2cycle
 *
 * @param triacPwm: triac related struct
 * @retval none
 */

void OD_PWM_CalcTriacPWM(PWM *triacPwm) {

	if(triacPwm->mode == PHASE_MODULATION)
	{
		OD_PWM_CalcTriacPWM_PM(triacPwm);
	}
	else if (triacPwm->mode == FREQUENCY_MODULATION)
	{
		OD_PWM_CalcTriacPWM_FM(triacPwm);
	}
}

void OD_PWM_CalcTriacPWM_PM(PWM *triacPwm) {
	// DebugPrintf("calc triac freq %.2f \n", triacPwm->frequency);
	// DebugPrintf("calc cycle var %d\n", triacPwm->dutyCycleVar);
	if (triacPwm->frequency > 0.0f && triacPwm->dutyCycleVar > 0U) {

		triacPwm->period = (uint32_t) (1000000.0f / triacPwm->frequency);// triac pwm period in microsecond
		triacPwm->shiftBy = (triacPwm->period - triacPwm->zcDelayAdjust);
		triacPwm->onDuration = (uint32_t) ((triacPwm->period
				* triacPwm->dutyCycleVar) / 100U);// PWM on duration time in microsecond
		triacPwm->offDuration = (triacPwm->period - triacPwm->onDuration);// PWM off duration time in microsecond
		// DebugPrintf("p: %d onD: %d offD: %d sby: %d \n",triacPwm->period,triacPwm->onDuration ,triacPwm->offDuration,triacPwm->shiftBy );
		triacPwm->period += triacPwm->shiftBy;
		triacPwm->offDuration += triacPwm->shiftBy;

		// DebugPrintf("p: %d onD: %d offD: %d sby: %d \n",triacPwm->period,triacPwm->onDuration ,triacPwm->offDuration,triacPwm->shiftBy );


	}


}

void OD_PWM_CalcTriacPWM_FM(PWM *triacPwm)
{
	if(triacPwm->frequency > 0 && triacPwm-> totalNumberOfCycle > 0 && triacPwm-> dutyCycle >0)
	{
		triacPwm->numberOfInactiveCycle = triacPwm->totalNumberOfCycle - triacPwm->numberOfActiveCycle;
		triacPwm->inactiveCycleFactor = triacPwm-> totalNumberOfCycle * 1.0/ triacPwm->numberOfInactiveCycle;
//		DebugPrintf("factor %f\r\n", triacPwm->inactiveCycleFactor);
		if(triacPwm->totalPulse> 0 && triacPwm-> dutyCycle >0)
		{
			triacPwm->onPulse = (uint32_t) (triacPwm->dutyCycle *triacPwm->totalPulse / 100.0);
			triacPwm->offPulse = triacPwm->totalPulse - triacPwm->onPulse;
			// DebugPrintf("on:%d of %d t: %d\n",triacPwm->onPulse,triacPwm->offPulse,triacPwm->totalPulse);

		}
	}
}




/**
 * @brief regulates pwm for triac 1 & 2
 * @param triacPwm: triac related struct
 * @retval none
 */
void OD_PWM_RegulateTriacPWM(PWM *triacPwm) {

	if(triacPwm->mode == PHASE_MODULATION)
	{
		OD_PWM_RegulateTriacPWM_PM(triacPwm);
	}
}

void OD_PWM_RegulateTriacPWM_PM(PWM *triacPwm) {

	if (triacPwm->enabled > 0U && triacPwm->frequency > 0.0f) {
		
		if ((triacPwm->dutyCycleVar < triacPwm->dutyCycle)
				&& (triacPwm->dutyCycleVar != triacPwm->dutyCycle)) {
			triacPwm->dutyCycleVar++;
		} else if ((triacPwm->dutyCycleVar > triacPwm->dutyCycle)
				&& (triacPwm->dutyCycleVar != triacPwm->dutyCycle)) {
			triacPwm->dutyCycleVar--;
		}
	} else {
		if (triacPwm->dutyCycleVar > 0U) {
			triacPwm->dutyCycleVar--;
		}
	}
	// DebugPrintf("D: %d ",triacPwm->dutyCycle);
	// DebugPrintf(",  DV: %d\r\n", triacPwm->dutyCycleVar);


}

/**
 * @brief generate pwm for triac 1&2
 * @param triacPwm: triac related struct
 * @param triacIndex: 2 instance of triac 1 and 2
 * @retval none
 */
void OD_PWM_GenTriacPWM(TIM_HandleTypeDef *htim, PWM *triacPwm,
		uint8_t triacIndex) {


	if(triacPwm->mode == PHASE_MODULATION)
	{
		OD_PWM_GenTriacPWM_PM(htim, triacPwm,triacIndex);
	}
	else if (triacPwm->mode == FREQUENCY_MODULATION)
	{
		OD_PWM_GenTriacPWM_FM(htim, triacPwm,triacIndex);
	}
}

// uint16_t buf1[500], buf2[500], index = 0, c1 = 0, c2 = 0;


void OD_PWM_GenTriacPWM_PM(TIM_HandleTypeDef *htim, PWM *triacPwm,
		uint8_t triacIndex) {
		
		triacPwm->numberOfActiveCycle++;
		

	if ((triacPwm->enabled > 0U || triacPwm->dutyCycleVar > 0U)) {// if triac power is on from HMI

		if (triacPwm->isZeroCrossed == 1U) {// check if zero crossed is happend or not
			triacPwm->isZeroCrossed = 2U;				// reset zero cross flag
			triacPwm->counter1 = 0U;

			// triacPwm->numberOfInactiveCycle++;
			// if(triacPwm->numberOfInactiveCycle == 1)
			// {
			// 	DebugPrintf("%d\n", triacPwm->numberOfActiveCycle);
			// 	triacPwm->numberOfActiveCycle = 0;
			// 	triacPwm->numberOfInactiveCycle = 0;
			// }
			// c2++;
			// buf1[index]=-1;
			// buf2[index]= -1;
			// index++;

			
		}
		if (triacPwm->isZeroCrossed >= 2U) {
			if (triacPwm->dutyCycleVar <= OD_PWM_TRIAC_DUTY_LOWER_LIMIT) {// if duty cycle less than or equal lower limit, reset the pwm channel
				//HAL_GPIO_WritePin(triacPwm->channel_1_port, triacPwm->channel_1, GPIO_PIN_RESET);
				triacIndex == TRIAC_1_INDEX ?
						HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin,
								GPIO_PIN_RESET) :
						HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin,
								GPIO_PIN_RESET);
			} else if (triacPwm->dutyCycleVar >= OD_PWM_TRIAC_DUTY_UPPER_LIMIT) {// if duty cycle less than or equal upper limit, set the pwm channel completely HIGH
				triacIndex == TRIAC_1_INDEX ?
						HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin,
								GPIO_PIN_SET) :
						HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin,
								GPIO_PIN_SET);
			}
			else
			{// if duty cycle is within 0 to 100 %, the generate the 100uspulse
				
				// counter1 for lower half cycle of the ac signal
				triacPwm->counter1 += triacPwm->incrementBy;// counter counts in microsecond, so 10kHz timer increment by 100us
//				if(index<500)
//				{
//				buf1[index]= triacPwm->counter1;
//				buf2[index]= triacPwm->counter2;
//				index++;
//				}
				// if(c2 == 15 )
				// {
				// 	DebugPrintf("....\n");
				// 	for(int i =0 ; i < 500; i++)
				// 	{
				// 		DebugPrintf("%d %d\n",buf1[i], buf2[i] );
				// 	}
				// 	DebugPrintf("---\n");
				// 	for(int i =0 ; i < 400; i++)
				// 	{
				// 		buf1[i] = 0;
				// 		buf2[i] = 0;
				// 		index = 0;
				// 	}
				// 	c2= 0;
				// }
				// counter2 for upper half cycle of the ac signal
				if (triacPwm->counter1
						> (triacPwm->shiftBy + triacPwm->zcDelayAdjust)) {
					triacPwm->counter2 = triacPwm->counter1
							- (triacPwm->shiftBy + triacPwm->zcDelayAdjust);
				} else {
					triacPwm->counter2 += triacPwm->incrementBy;
				}

				// generates pwm for lower half cycle
				if ((triacPwm->counter1 > triacPwm->offDuration)
						&& (triacPwm->counter1 < triacPwm->period)) {// generate an impulse to trigger the triac
					if (triacIndex == TRIAC_1_INDEX) {
						//generates 100us pulse to trigger the triac 1
						if (triacPwm->counter1
								> (triacPwm->offDuration + triacPwm->pulse)) { // generate an impulse for 100us
							HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin,
									GPIO_PIN_RESET);
						} else {
							HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin,
									GPIO_PIN_SET);
						}
					} else {
						//generates 500us pulse to trigger the triac 2
						if (triacPwm->counter1
								> (triacPwm->offDuration + triacPwm->pulse)) {
							HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin,
									GPIO_PIN_RESET);
						} else {
							HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin,
									GPIO_PIN_SET);
						}
					}
				}

				// generates pwm for upper half cycle
				if ((triacPwm->counter2 > triacPwm->offDuration)
						&& (triacPwm->counter2 < triacPwm->period)) {// generate an impulse to trigger the triac
					if (triacIndex == TRIAC_1_INDEX) {
						//generates 500us pulse to trigger the triac 1
						if (triacPwm->counter2
								> (triacPwm->offDuration + triacPwm->pulse)) { // generate an impulse for 500us
							HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin,
									GPIO_PIN_RESET);
						} else {
							HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin,
									GPIO_PIN_SET);
						}
					} else {
						//generates 500us pulse to trigger the triac 2
						if (triacPwm->counter2
								> (triacPwm->offDuration + triacPwm->pulse)) {
							HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin,
									GPIO_PIN_RESET);
						} else {
							HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin,
									GPIO_PIN_SET);
						}
					}
				}
			}
		}
	} else {
		triacPwm->counter1 = 0U;
		triacPwm->counter2 = 0U;
	}
}


void OD_PWM_GenTriacPWM_FM(TIM_HandleTypeDef *htim, PWM *triacPwm , uint8_t triacIndex)
{
	triacPwm->pulseCounter++;
	if(triacPwm->isZeroCrossed == 1)
	{
		triacPwm->isZeroCrossed = 2U;
		triacPwm->totalPulse = triacPwm-> pulseCounter;
		// DebugPrintf("pC: %d\n", triacPwm->pulseCounter);
		triacPwm-> pulseCounter= 0U;
	

		if(triacPwm->enabled> 0 
				&& triacPwm->inactiveCycleFactor > 0 
				&& triacPwm->numberOfActiveCycle != TOTAL_NUMBER_OF_CYCLE)
		{

			if(triacPwm->currentCyclePos > triacPwm->nextInactiveCyclePos /*&& triacPwm->nextInactiveCycleFlag == 1*/)
			{
				triacPwm->nextInactiveCycleFrac = triacPwm->nextInactiveCycleFrac + triacPwm->inactiveCycleFactor ;
				triacPwm->nextInactiveCyclePos = (uint16_t)(triacPwm->nextInactiveCycleFrac);
				triacPwm->nextInactiveCycleFlag = 2U;
				if(triacPwm->nextInactiveCyclePos == TOTAL_NUMBER_OF_CYCLE)
				{
					triacPwm->nextInactiveCycleFrac = 0;
					triacPwm->nextInactiveCyclePos = 0;
				}
				DebugPrintf("NInac:%d\n", triacPwm->nextInactiveCyclePos);
			}
			DebugPrintf("P:%d\n",triacPwm->currentCyclePos);
		}
	}


		if(triacPwm->enabled> 0  
		 && triacPwm->currentCyclePos == triacPwm->nextInactiveCyclePos
		 && triacPwm->numberOfActiveCycle != TOTAL_NUMBER_OF_CYCLE)
			{
				triacPwm->nextInactiveCycleFlag = 1U;
				// inactive triac
				// DebugPrintf("inac pos: %d\n",triacPwm->currentCyclePos);
				HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin, GPIO_PIN_RESET);

			}
			else
			{
				// active triac
				if(triacPwm->pulseCounter > triacPwm-> offPulse && triacPwm->pulseCounter < triacPwm->totalPulse)
				{
					if(triacPwm->pulseCounter > (triacPwm-> offPulse + 20))
					{
						HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin, GPIO_PIN_SET);
					}
					else
					{
						HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin, GPIO_PIN_RESET);
					}
			}
			else
			{
				HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin, GPIO_PIN_RESET);
			}
				// DebugPrintf("Actv: %d - NInac = %d\n",triacPwm->currentCyclePos, triacPwm->nextInactiveCyclePos);
//				DebugPrintf("next inactive: %d\r\n",triacPwm->nextInactiveCyclePos);
			}


	if(triacPwm->enabled> 0 && triacPwm->numberOfActiveCycle == TOTAL_NUMBER_OF_CYCLE && triacPwm->dutyCycle !=0)
		{
			if(triacPwm->pulseCounter > triacPwm-> offPulse && triacPwm->pulseCounter < triacPwm->totalPulse)
			{
				if(triacPwm->pulseCounter > (triacPwm-> offPulse + 20))
				{
					HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin, GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin, GPIO_PIN_RESET);
				}
			}
			else
			{
				HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin, GPIO_PIN_RESET);
			}
			// HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin, GPIO_PIN_SET);
			// DebugPrintf("always on\n");
		}
		else if (triacPwm->enabled> 0 && (triacPwm->numberOfActiveCycle == 0 || triacPwm->dutyCycle ==0))
		{
			HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin, GPIO_PIN_RESET);
			// DebugPrintf("always off\n");
		}
}


