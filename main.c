/*
 * main.c
 *
 *  Created on: 24 Mar 2022
 *      Author: josiah chua
 */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "Modes.h"
#include "Other_Functions.h"
#include "stdio.h"
#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"


int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	Enable_AWT_Accelerometer();

	Enable_Door_Sensor();

	UART1_Init();

	BSP_LED_Init(LED2);

	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	/* Peripheral initializations using BSP functions */
	BSP_PSENSOR_Init();
	BSP_TSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();

	uint32_t previous_time=0U;

    while (1)
    {
    	uint32_t current_time=HAL_GetTick();
    	if (current_time-previous_time>=1000U)
    	{
    		if(get_Warning_Mode()==WARNING_ON)
    		{
    			Warning_Mode_Output();
    		}
    		else
    		{

				if (get_Mode()!=get_Check_Mode())
				{
					init_new_mode(get_Mode());
				}
				if (get_Mode()==STATIONARY)
				{
					if(get_If_Count_Down()==TRUE)
					{
						Stationary_Mode_Count_Down();
					}
					else
					{
						Stationary_Mode_Readings();
					}
				}
				else if (get_Mode()==LAUNCH)
				{
					Launch_Return_Mode_Readings();
				}
				else if (get_Mode()==RETURN)
				{
					Launch_Return_Mode_Readings();
				}
    		}
    		previous_time=current_time;
    	}
    	if (check_prev_Door_Open()!=check_Door_Open())
    	{
        	if (check_Door_Open()==TRUE)
        	{
        		char message[16];
        		sprintf(message,"Doors Opening\r\n");
        		transmit_to_TermiNUS((uint8_t*) message);
        		update_prev_Door_Open();

        	}
        	else
        	{
        		char message[16];
    			sprintf(message,"Door Closing\r\n");
    			transmit_to_TermiNUS((uint8_t*) message);
    			update_prev_Door_Open();
        	}
    	}

    }
}

