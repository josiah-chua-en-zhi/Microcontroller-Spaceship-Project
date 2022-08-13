/*
 * Modes.c
 *
 *  Created on: 24 Mar 2022
 *      Author: josiah chua
 */

/* Includes ------------------------------------------------------------------*/
#include "Modes.h"
#include "Other_Functions.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/Components/lsm6dsl/lsm6dsl.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
#include "stdio.h"
#include "string.h"

//Initialize Mode
volatile Mode_TypeDef Mode=STATIONARY;
volatile Mode_TypeDef Previous_Mode=RETURN;
volatile Mode_TypeDef Next_Mode=LAUNCH;
volatile Warning_TypeDef Warning_Mode=WARNING_OFF;
volatile Mode_TypeDef Check_Mode=RETURN;
volatile Boolean In_Count_Down=FALSE;
volatile Boolean Door_Open=FALSE;
volatile Boolean Start_Door_Timer=FALSE;
volatile Boolean prev_Door_Open=FALSE;

int count_down=9;

//Threshold Values
double wetbulb_max_threshold = 35.00;
float temp_max_threshold = 40.00;
float temp_min_threshold = 4.00;
float pres_min_threshold = 750.00;
float angular_rate_threshold = 10.00;

//debugging settings.
#define ON 1
#define OFF 0
#define DEBUG_MODE OFF // set to ON when debugging. For other settings, turn on to change the inputs in debug mode
#define DEBUG_temp OFF // turn on to modify temperature values
#define DEBUG_pres OFF
#define DEBUG_hum OFF
#define DEBUG_accel OFF
#define DEBUG_gyro OFF
#define DEBUG_magneto OFF
#define WARNINGS_ENABLED ON


Mode_TypeDef get_Mode(void)
{
	return Mode;
}

Mode_TypeDef get_Check_Mode(void)
{
	return Check_Mode;
}

void update_Check_Mode(void)
{
	Check_Mode=Mode;
}

void update_Mode(void)
{
	Mode_TypeDef temp=Previous_Mode;
	Previous_Mode=Mode;
	Mode=Next_Mode;
	Next_Mode=temp;
}

Warning_TypeDef get_Warning_Mode(void)
{
	return Warning_Mode;
}

void update_Warning_Mode(void)
{
	if(Warning_Mode==WARNING_OFF){
		Warning_Mode=WARNING_ON;
		change_duty_cycle(get_Mode());
		reset_Count_Down();
	}
	else{
		Warning_Mode=WARNING_OFF;
	}
}


void Warning_Mode_Output(void)
{
	char message[64];
	if (get_Mode()==STATIONARY){
		sprintf(message,"STATIONARY mode: WARNING\r\n");
	}
	else if (get_Mode()==LAUNCH)
	{
		sprintf(message,"LAUNCH mode: WARNING\r\n");
	}
	else
	{
		sprintf(message,"RETURN mode: WARNING\r\n");
	}
	transmit_to_TermiNUS((uint8_t*) message);
}


void init_new_mode(Mode_TypeDef new_mode)
{
	char message[64];
	if (new_mode==STATIONARY){
		sprintf(message,"Entering STATIONARY mode\r\n");
	}
	else if (new_mode==LAUNCH)
	{
		sprintf(message,"Entering LAUNCH mode\r\n");
	}
	else
	{
		sprintf(message,"Entering RETURN mode\r\n");
	}
	transmit_to_TermiNUS((uint8_t*) message);
	change_duty_cycle(new_mode);
	Check_Mode=Mode;
}

Boolean get_If_Count_Down(void)
{
	return In_Count_Down;
}

void set_Count_Down(void)
{
	In_Count_Down=TRUE;
}

void reset_Count_Down(void)
{
	count_down=9;
	In_Count_Down=FALSE;
}

Boolean check_Door_Open(void)
{
	return Door_Open;
}

Boolean check_prev_Door_Open(void)
{
	return prev_Door_Open;
}

void close_door(void)
{
	Door_Open=FALSE;
}

void open_door(void)
{
	Door_Open=TRUE;
}

Boolean check_Start_Door_Timer(void)
{
	return Start_Door_Timer;
}

void update_Start_Door_Timer(void)
{
	if(Start_Door_Timer==TRUE)
	{
		Start_Door_Timer=FALSE;
	}
	else
	{
		Start_Door_Timer=TRUE;
	}
}

void update_prev_Door_Open(void)
{
	if(prev_Door_Open==TRUE){
		prev_Door_Open=FALSE;
	}
	else{
		prev_Door_Open=TRUE;
	}
}


void Stationary_Mode_Count_Down(void)
{
	char message[4];
	sprintf(message,"%d, ",count_down);
	transmit_to_TermiNUS((uint8_t*) message);
	Stationary_Mode_Readings();
	if(count_down==0)
	{
		reset_Count_Down();
		update_Mode();
	}
	else
	{
		count_down--;
	}
}

void Stationary_Mode_Readings(void)
{
	//Poll data from Temperature, Pressure and Humidity sensors
	float pres_data = BSP_PSENSOR_ReadPressure();
	float temp_data = BSP_TSENSOR_ReadTemp();
	float hum_data = BSP_HSENSOR_ReadHumidity();
	double wetbulb = get_wetbulb_temp((double) temp_data, (double) hum_data);

	//Format message to TermiNUS
	char message[128];
	sprintf(message,"T:%.2f, P:%.2f, H:%.2f, WB:%.2f\r\n",temp_data, pres_data,hum_data, wetbulb);
	transmit_to_TermiNUS((uint8_t*) message);

	//Debugger Mode: override data with TermiNUS inputs
	if (DEBUG_temp == ON)
	{
		sprintf(message,"DEBUG MODE: Enter temperature (deg celc) override in .2 float (20 char max): \r\n");
		transmit_to_TermiNUS((uint8_t*) message);
		temp_data = float_terminal_override();
	}

	if (DEBUG_pres == ON)
	{
		sprintf(message,"DEBUG MODE: Enter pressure (hPa) override in .2 float (20 char max): \r\n");
		transmit_to_TermiNUS((uint8_t*) message);
		pres_data = float_terminal_override();
	}

	if (DEBUG_hum == ON)
	{
		sprintf(message,"DEBUG MODE: Enter humidity(%%rH) override in .2 float (20 char max): \r\n");
		transmit_to_TermiNUS((uint8_t*) message);
		hum_data = float_terminal_override();
	}
	if (DEBUG_MODE == ON)
	{
		sprintf(message,"Debugger Mode: Presenting updated data \r\n");
		transmit_to_TermiNUS((uint8_t*) message);

		wetbulb = get_wetbulb_temp((double) temp_data, (double) hum_data);

		sprintf(message,"T:%.2f, P:%.2f, H:%.2f, WB:%.2f\r\n",temp_data, pres_data, hum_data, wetbulb);
		transmit_to_TermiNUS((uint8_t*) message);
	}

	// Compare data against threshold values: determine the need to enter Warning Mode
	if(WARNINGS_ENABLED==ON)
	{
		if (temp_data > temp_max_threshold || temp_data < temp_min_threshold || wetbulb > wetbulb_max_threshold || pres_data < pres_min_threshold)
		{
			update_Warning_Mode();
		}
	}

}

void Launch_Return_Mode_Readings(void)
{
	//Poll data from Temperature, Pressure and Humidity sensors
	float pres_data = BSP_PSENSOR_ReadPressure();
	float temp_data = BSP_TSENSOR_ReadTemp();
	float hum_data = BSP_HSENSOR_ReadHumidity();
	double wetbulb = get_wetbulb_temp((double) temp_data, (double) hum_data);

	//Poll data from Accelerometer measurements
	float accel_data[3];
	int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
	BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
	// the function above returns 16 bit integers which are 102 * acceleration in m/s^2 (to verify).
	//Converting to float to print the actual acceleration.
	accel_data[0] = (float) accel_data_i16[0] / 102.0f;
	accel_data[1] = (float) accel_data_i16[1] / 102.0f;
	accel_data[2] = (float) accel_data_i16[2] / 102.0f;

	//poll data from gyroscope measurements
	float gyro_data[3];
	float gyro_data_i16[3] = { 0 }; // array to store the x, y and z readings.
	BSP_GYRO_GetXYZ(gyro_data_i16);
	gyro_data[0] = (float)gyro_data_i16[0] / 1000.0f; // pitch
	gyro_data[1] = (float)gyro_data_i16[1] / 1000.0f; // roll
	gyro_data[2] = (float)gyro_data_i16[2] / 1000.0f; //yaw

	// poll data from magnetometer measurements
	float magneto_data[3];
	int16_t magneto_data_i16[3] = { 0 }; // array to store x,y, and z readings.
	BSP_MAGNETO_GetXYZ(magneto_data_i16);
	// function above returns 16 bit integers which are ???.
	//Converting to float to print the actual magnetic field strength:
	magneto_data[0] = (float)magneto_data_i16[0] / 1000.0f;
	magneto_data[1] = (float)magneto_data_i16[1] / 1000.0f;
	magneto_data[2] = (float)magneto_data_i16[2] / 1000.0f;

	//Format message to TermiNUS
	char message[128];
	sprintf(message,"T:%.2f, P:%.2f, H:%.2f, WB:%.2f,A:%.2f, %.2f, %.2f G: %.2f, %.2f, %.2f M: %.2f, %.2f, %.2f\r\n",
			temp_data,pres_data,hum_data,wetbulb,
			accel_data[0], accel_data[1], accel_data[2],
			gyro_data[0], gyro_data[1], gyro_data[2],
			magneto_data[0],magneto_data[1],magneto_data[2]);
	transmit_to_TermiNUS((uint8_t*) message);

	//Debugger Mode: override data with TermiNUS inputs
	if (DEBUG_temp == ON)
		{
			sprintf(message,"DEBUG MODE: Enter temperature (deg celc) override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			temp_data = float_terminal_override();
		}

	if (DEBUG_pres == ON)
		{
			sprintf(message,"DEBUG MODE: Enter pressure (hPa) override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			pres_data = float_terminal_override();
		}

	if (DEBUG_hum == ON)
		{
			sprintf(message,"DEBUG MODE: Enter humidity(%% rH) override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			hum_data = float_terminal_override();
		}

	if (DEBUG_accel == ON)
		{
			sprintf(message,"DEBUG MODE: Enter accel_x override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			accel_data[0] = float_terminal_override();

			sprintf(message,"DEBUG MODE: Enter accel_y override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			accel_data[1] = float_terminal_override();

			sprintf(message,"DEBUG MODE: Enter accel_z override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			accel_data[2] = float_terminal_override();
		}
	if (DEBUG_gyro == ON)
		{
			sprintf(message,"DEBUG MODE: Enter gyro_x override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			gyro_data[0] = float_terminal_override();

			sprintf(message,"DEBUG MODE: Enter gyro_y override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			gyro_data[1] = float_terminal_override();

			sprintf(message,"DEBUG MODE: Enter gyro_z override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			gyro_data[2] = float_terminal_override();
		}

	if (DEBUG_magneto == ON)
		{
			sprintf(message,"DEBUG MODE: Enter magneto_x override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			magneto_data[0] = float_terminal_override();

			sprintf(message,"DEBUG MODE: Enter magneto_y override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			magneto_data[1] = float_terminal_override();

			sprintf(message,"DEBUG MODE: Enter magneto_z override in .2 float (20 char max): \r\n");
			transmit_to_TermiNUS((uint8_t*) message);
			magneto_data[2] = float_terminal_override();
		}

	if (DEBUG_MODE == ON)
		{
			sprintf(message,"Debugger Mode: Presenting updated data \r\n");
			transmit_to_TermiNUS((uint8_t*) message);

			wetbulb = get_wetbulb_temp((double) temp_data, (double) hum_data);

			sprintf(message,"T:%.2f, P:%.2f, H:%.2f, WB:%.2f, A:%.2f, %.2f, %.2f G: %.2f, %.2f, %.2f M: %.2f, %.2f, %.2f\r\n",
			temp_data,pres_data,hum_data, wetbulb,
			accel_data[0], accel_data[1], accel_data[2],
			gyro_data[0], gyro_data[1], gyro_data[2],
			magneto_data[0],magneto_data[1],magneto_data[2]);

			transmit_to_TermiNUS((uint8_t*) message);
		}

	// Compare data against threshold values: determine the need to enter Warning Mode
	int warning = 0;
	if (temp_data > temp_max_threshold || temp_data < temp_min_threshold || wetbulb > wetbulb_max_threshold)
		warning = 1;
	if (pres_data < pres_min_threshold)
		warning = 1;
	if (gyro_data[0] > angular_rate_threshold || gyro_data[0] < (0 - angular_rate_threshold) || gyro_data[1] > angular_rate_threshold || gyro_data[1] < (0 - angular_rate_threshold) || gyro_data[2] > angular_rate_threshold || gyro_data[2] < (0 - angular_rate_threshold))
		warning = 1;

	if(WARNINGS_ENABLED==ON)
	{
		if (warning)
		{
			update_Warning_Mode();
		}
	}

}


void Set_Register(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	uint8_t ctrl;
	ctrl = SENSOR_IO_Read(Addr, Reg);
	ctrl |= Value;
	SENSOR_IO_Write(Addr, Reg, ctrl);
}

void Reset_Register(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	uint8_t ctrl;
	ctrl = SENSOR_IO_Read(Addr, Reg);
	ctrl &= Value;
	SENSOR_IO_Write(Addr, Reg, ctrl);
}

void Enable_AWT_Accelerometer(void)
{
	// Enable AHB2 Bus for GPIOD
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t PP=0b0001000, SP=0b0;
	HAL_NVIC_SetPriority(LSM6DSL_INT1_EXTI11_EXTI_IRQn, PP , SP);
	NVIC_ClearPendingIRQ(LSM6DSL_INT1_EXTI11_EXTI_IRQn);
	NVIC_EnableIRQ(LSM6DSL_INT1_EXTI11_EXTI_IRQn);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = LSM6DSL_INT1_EXTI11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &GPIO_InitStruct);

	//enable free fall function and interrupt
	Set_Register(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, 0x80);

	//FF time of 1 sec. Threshold is the minimum default
	Set_Register(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WAKE_UP_DUR, 0x80);
	Set_Register(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FREE_FALL, 0xA0);

	//Testing
	//Set_Register(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FREE_FALL, 0x0A);

	//routing free fall interrupt to int1 pin
	Set_Register(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, 0x10);


	/*
	Tilt Interrupt
	Set_Register(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, 0x02);

	Free Fall interrupt
	//enable free fall function
	Set_Register(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, 0x80);

	//FF time of 1 sec. Threshold is the minimum default
	Set_Register(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WAKE_UP_DUR, 0x80);
	Set_Register(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FREE_FALL, 0xA0);

	//routing free fall interrupt to int1 pin
	Set_Register(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, 0x10);
	*/

}


//door sensor code


void Enable_Door_Sensor(void)
{
	// Enable AHB2 Bus for GPIOD
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t PP=0b0010000, SP=0b0;
	HAL_NVIC_SetPriority(ARD_D2_EXTI_IRQn, PP , SP);
	NVIC_ClearPendingIRQ(ARD_D2_EXTI_IRQn);
	NVIC_EnableIRQ(ARD_D2_EXTI_IRQn);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ARD_D2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ARD_D2_GPIO_Port, &GPIO_InitStruct);
}



