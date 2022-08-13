/*
 * main.c
 *
 *  Created on: 24 Mar 2022
 *      Author: josiah chua
 */
/**

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "stdio.h"
#include "stm32l4xx_it.h"
#include "string.h"
#include "stdlib.h"
#include "Modes.h"
#include "Other_Functions.h"
#include "../../Drivers/BSP/Components/lsm6dsl/lsm6dsl.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"

// initializing UART structure
UART_HandleTypeDef huart1;

//Function to transmit data to TermiNUS
void transmit_to_TermiNUS(uint8_t* message)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen((const char*)message),10000);
}

//Function to receive float data during debug mode
// input is in human-readable float data, but no guarantees on the output for ridiculous input. 20 characters max.
float float_terminal_override(void)
{
	//taking user input
	char readback[64];
	char message[64]; //data buffer
	int i = 0;
	while(i < 20)
	    {
	    HAL_UART_Receive(&huart1, (uint8_t*) &message[i], (uint16_t) 1, 0xFFFF);
	    if (message[i] == '\r') break;
	    else i++;
	    }
	//splicing the message
	char *cleaned_message;
	cleaned_message = malloc(i * sizeof(char));
	for(int j=0;j<i;j++)
	   cleaned_message[j] = message[j];

	//char* to float conversion
	float float_message = atof(cleaned_message);

	//readback to confirm correct values
	sprintf(readback, "%.2f \r\n",float_message);
	HAL_UART_Transmit(&huart1, (uint8_t*) readback, strlen((const char*)readback),10000);
	return float_message;
}


//Function to initialize UART transmission
void UART1_Init(void)
{
    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configuring UART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

}

//LED blinking functions

/*
Variables to change blinking type
Each tick is 1ms so set blink_off and blink_on respectively to adjust duty cycle
*/
volatile int blink_frequency=0;
volatile int blink_on=0;
volatile int blink_off=0;
volatile Current_State_TypeDef LED_Current_State=BLINK_ON;


/*
Get the current duration of waiting before LED will be toggled during SYSTick interrupt
 */
int get_current_wait(void)
{
	// return the opposite of the previous waiting duration
	if(LED_Current_State==BLINK_ON)
	{
		return blink_on;
	}
	else
	{
		return blink_off;
	}
}

Current_State_TypeDef get_LED_Current_State(void)
{
	// return current state
	return LED_Current_State;

}


/*
Update the previous duration of waiting to determine the current duration
of waiting before LED will be toggled during SYSTick interrupt
 */
void update_Current_State(void)
{
	// Update previous_waiting duration with the opposite of it(the current waiting duration)
	if(LED_Current_State==BLINK_ON)
	{
		LED_Current_State=BLINK_OFF;
	}
	else
	{
		LED_Current_State=BLINK_ON;
	}
}


/*
Change blink variable to vary duty cycle
 */
void change_duty_cycle(Mode_TypeDef mode)
{
	if(get_Warning_Mode()==WARNING_ON)
	{
		blink_frequency=FREQUENCY_2HZ;
		blink_on=DUTY_CYCLE_ON_50_PER/blink_frequency;
		blink_off=DUTY_CYCLE_OFF_50_PER/blink_frequency;
	}
	else
	{
		if(mode==STATIONARY)
		{
			//if it is stationary turn LED off
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			blink_frequency=FREQUENCY_1HZ;
			blink_on=DUTY_CYCLE_ON_0_PER/blink_frequency;
			blink_off=DUTY_CYCLE_OFF_0_PER/blink_frequency;
		}
		else if(mode==LAUNCH)
		{
			blink_frequency=FREQUENCY_1HZ;
			blink_on=DUTY_CYCLE_ON_25_PER/blink_frequency;
			blink_off=DUTY_CYCLE_OFF_25_PER/blink_frequency;
		}
		else if(mode==RETURN)
		{
			blink_frequency=FREQUENCY_1HZ;
			blink_on=DUTY_CYCLE_ON_75_PER/blink_frequency;
			blink_off=DUTY_CYCLE_OFF_75_PER/blink_frequency;
		}
	}
}

//Set interrupt callback handler
volatile WarningTimer_TypeDef Warning_Timer=TIMER_STOP;
volatile uint32_t prev_time=0U;

uint32_t get_prev_time(void){
	return prev_time;
}

WarningTimer_TypeDef get_Warning_Timer(void){
	return Warning_Timer;
}

void change_Warning_Mode_Timer(void){
	if (get_Warning_Timer()==TIMER_STOP){
		Warning_Timer=TIMER_START;
	}
	else{
		Warning_Timer=TIMER_STOP;
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	//interrupt hander for push buttons to change mode
	if(GPIO_Pin == BUTTON_EXTI13_Pin)
	{
		uint32_t current_time=HAL_GetTick();
		if (get_Warning_Mode()==WARNING_ON)
		{
			change_Warning_Mode_Timer();
		}
		else
		{
			if(current_time-get_prev_time() <1000)
			{
				if(get_Mode()==STATIONARY){
					set_Count_Down();
				}
				else
				{
					update_Mode();
				}
				prev_time=0;
			}
			else{
				prev_time=current_time;

			}
		}
	}
	if(GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin)
	{
		//Read free fall status pin to check if activated
		uint8_t ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WAKE_UP_SRC);
		ctrl |= 0xDF;

		if (ctrl && 0xFF)
		{
			if (get_Warning_Mode()!=WARNING_ON)
			{
				update_Warning_Mode();
			}
		}
	}
	if(GPIO_Pin == ARD_D2_Pin)
	{
		if (check_Door_Open()==FALSE)
		{
			open_door();
		}
		else
		{
			update_Start_Door_Timer();
			reset_door_timer();
		}
	}

}

// Tw = T * arctan[0.152 * (rh + 8.3136)^(1/2)] + arctan(T + rh%)
// – arctan(rh – 1.6763) + 0.00391838 *(rh)^(3/2) * arctan(0.0231 * rh) – 4.686
double get_wetbulb_temp (double T, double rh)
{
	return T + atan(0.152 * sqrt(rh + 8.3136)) + atan(T + rh / 100) - atan(rh - 1.6763) + 0.00391838 * sqrt(rh) * sqrt(rh) * sqrt(rh) * atan(0.0231 * rh) - 4.686;
}



