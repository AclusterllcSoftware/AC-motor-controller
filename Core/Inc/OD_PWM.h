/*
 * PWM_Handler.h
 *
 *  Created on: Mar 13, 2022
 *      Author: RASEL_EEE
 */

#ifndef INC_OD_PWM_H_
#define INC_OD_PWM_H_

#include "main.h"


//#define PWM_POLARITY_INVERSE

#define OD_PWM_MAX_VALUE 10U		// maximum value of the PWM
#define OD_PWM_TRIAC_TRIG_IMPULSE_DURATION		1U // 1x100 = 100 microsecond
#define OD_PWM_TRIAC_DUTYCYCLE_OFFSET			0U
//#define OD_PWM_TRIAC_ZC_DETEC_DELAY				1300U	//there is delay between ac signal zero cross and zero cross detection by 1300us
#define OD_PWM_TRIAC_DUTY_UPPER_LIMIT			95U		// if duty cycle is greater than upper limit the duty cycle will be 100
#define OD_PWM_TRIAC_DUTY_LOWER_LIMIT			5U		// if duty cycle is greater than lower limit, the duty cycle will be 0

//
//typedef enum PWM_Source{
//
//}PWM_Source;


enum PWM_Mode {
    PHASE_MODULATION,
    FREQUENCY_MODULATION
};

typedef struct PWM{
	uint8_t enabled;
	uint8_t mode;
	uint32_t counter1;
	uint32_t counter2;					// counter2 will be used only for triac 1 & 2
	uint8_t isCounter2Active;			// isCounter2Active will be used only for triac 1 & 2
	uint32_t shiftBy;					// shiftBy will be used only for triac 1 & 2
	uint16_t incrementBy;				// pwm counter will increment by the value of this variable
	uint32_t pulse;						// pulse duration to trigger the triac
	float frequency;					//PWM frequency
	uint32_t onDuration;				//On duration
	uint32_t offDuration;				//Off duration
	uint32_t period;					//Cycle Time
	uint8_t dutyCycle;
	uint8_t lastDutyCycle;
	uint8_t dutyCycleVar;				// duty cycle var is used to increment or decrement the duty cycle
	uint8_t isZeroCrossed;
	uint32_t zcDelayAdjust;			//in us, zero cross detection of ac signal have some delay time, so we have to adjust it.
	GPIO_TypeDef *channel_1_port;
	uint16_t channel_1;
	GPIO_TypeDef *channel_2_port; // not used mir
	uint16_t channel_2;
	uint8_t excepStatus;				// exception status of the timer which generating the PWM


	uint32_t zeroCrossCounter;
	uint16_t totalNumberOfCycle;
	uint16_t numberOfActiveCycle;
	uint16_t numberOfInactiveCycle;
	uint16_t currentCyclePos;
	uint16_t nextInactiveCyclePos;
	uint8_t nextInactiveCycleFlag;
	float nextInactiveCycleFrac;
	float inactiveCycleFactor;
	uint32_t pulseCounter;
	uint32_t onPulse;
	uint32_t offPulse;
	uint32_t totalPulse;

}PWM;



//typedef struct FM{
//	uint8_t enabled;
//	uint8_t isZeroCrossed;
//	uint32_t counter1;
//	uint16_t totalNumberOfCycle;
//	uint16_t numberOfActiveCycle;
//	uint16_t numberOfInactiveCycle;
//	uint16_t currentCyclePos;
//	uint16_t nextInactiveCyclePos;
//	uint8_t nextInactiveCycleFlag;
//	float nextInactiveCycleFrac;
//	float frequency;
//	float inactiveCycleFactor;
//
//}FM;

/*----------- PWM for triac 1 & 2-----------*/
void OD_PWM_InitTriac(TIM_HandleTypeDef *htim, PWM *triacPwm, uint16_t acLineFrequency);
void OD_PWM_CalcTriacPWM(PWM *triacPwm);
void OD_PWM_RegulateTriacPWM(PWM *triacPwm);
void OD_PWM_GenTriacPWM(TIM_HandleTypeDef *htim, PWM *triacPwm, uint8_t triacIndex);


void OD_PWM_InitTriac_PM(TIM_HandleTypeDef *htim, PWM *triacPwm, uint16_t acLineFrequency);
void OD_PWM_CalcTriacPWM_PM(PWM *triacPwm);
void OD_PWM_RegulateTriacPWM_PM(PWM *triacPwm);
void OD_PWM_GenTriacPWM_PM(TIM_HandleTypeDef *htim, PWM *triacPwm, uint8_t triacIndex);



void OD_PWM_InitTriac_FM(TIM_HandleTypeDef *htim, PWM *triacPwm, uint16_t acLineFrequency);
void OD_PWM_CalcTriacPWM_FM(PWM *triacPwm);
void OD_PWM_GenTriacPWM_FM(TIM_HandleTypeDef *htim, PWM *triacPwm , uint8_t triacIndex);


#endif /* INC_OD_PWM_H_ */
