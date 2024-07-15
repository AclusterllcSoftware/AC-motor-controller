#include "debug.h"
#include "main.h"
#include "test.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stdlib.h"

extern UART_HandleTypeDef huart1;
extern GVar_ts gVar;

void DebugPrintf(const char *fmt, ...) {
    static char buffer[256]; // Adjust buffer size as needed
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}




void getInputforDutyCycle()
{
	char input[50];
		HAL_StatusTypeDef status;

		DebugPrintf("Please Enter Duty Cycle: \r\n");

		// Receive input from the user with a timeout of 10 seconds
		status = HAL_UART_Receive(&huart1, (uint8_t*)input, sizeof(input), 10000);

		if (status == HAL_OK)
		{

			DebugPrintf("You entered %d \r\n", input);
#if defined(PWMSIGNAL)
			gVar.triac_1.dutyCycle =  atoi(input);
			gVar.triac_2.dutyCycle =  atoi(input);
#endif
		}
		else if (status == HAL_TIMEOUT)
		{
			DebugPrintf("you have not entered any input so default value is 30 \r\n");
#if defined(PWMSIGNAL)
			gVar.triac_1.dutyCycle =  50;
			gVar.triac_2.dutyCycle =  50;
#endif
		}
}
