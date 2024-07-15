/*
 * Test.h
 *
 *  Created on: Apr 21, 2024
 *      Author: wsrra
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_
#include "OD_PWM.h"


#define CNFG_DEF_ZC_DELAY_ADJUST 		1300 //microsecond


#define MODULATION_TYPE					1
#define DUTYCYCLE						0 // for PM
#define NUMBER_OF_ACTIVE_CYCLE 			10 // for FM


#define TOTAL_NUMBER_OF_CYCLE 			50
#define MAX_ZERO_COUNT 					100 //TOTAL_NUMBER_OF_CYCLE*2
#define AC_SIGNAL_FREQ					50

///*----------- PWM for triac 1 & 2-----------*/
typedef enum TriacIndexEnum{
	TRIAC_1_INDEX = 1,
	TRIAC_2_INDEX,
	TRIAC_MAX_INDEX,
}TriacIndexEnum;



typedef struct GVar{

	PWM triac_1;
	PWM triac_2;
}GVar_ts;


void Test_Init(void);
void Test_Handler(void);


//void  ExecutionTimeCalculate();


#endif /* INC_TEST_H_ */
