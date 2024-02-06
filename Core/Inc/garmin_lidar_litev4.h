/*
 * garmin_lidar_litev4.h
 *
 */


#ifndef SRC_GARMIN_LIDAR_LITEV4_H_
#define SRC_GARMIN_LIDAR_LITEV4_H_

#include "stm32f3xx_hal_conf.h"
#include "RGB_LED.h"
#include <stdio.h>
#include <string.h>

#define TIMEOUT_MS 1000

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;


uint8_t LIDAR_ADDR1 = 0x62;
uint8_t LIDAR_ADDR2 = 0x55;
uint8_t ACQ_COMMAND = 0x00;
uint8_t TAKE_DIST = 0x04;
uint8_t DISTANCE_REG_LOW = 0x10;
uint8_t DISTANCE_REG_HIGH = 0x11;
uint8_t STATUS_REG = 0x01;
uint8_t BOARD_TEMPERATURE = 0xE0;
uint8_t FACTORY_RESET = 0xE4;
uint8_t DETECTION_SENSIVITY = 0x1C;
uint8_t ACQUISITION_COUNT = 0x05;
uint8_t QUICK_TERMINATION = 0xE5;
uint8_t SOC_TEMPERATURE = 0xEC;
uint8_t HIGH_ACCURACY_MODE = 0xEB;
uint8_t status;
uint8_t distance_low;
uint8_t distance_high;
uint16_t distanceL = 0;
uint16_t distanceR = 0;
char msg[128];

HAL_StatusTypeDef CheckDevice(uint8_t lidarAddr){
	char msg[128];

	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, lidarAddr << 1, 10, HAL_MAX_DELAY);
	if(ret == HAL_OK)
	{
	  sprintf(msg, "Device is ready.\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
	}
	else
	{
	  sprintf(msg, "Device is not connected.\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
	}
	return ret;
}

void configureLidarAddress(uint8_t lidarliteAddress, uint8_t newAddress, uint8_t disableDefault)
{
	uint8_t dataBytes[5];
    // Enable flash storage
    dataBytes[0] = 0x11;
    HAL_I2C_Mem_Write(&hi2c1, lidarliteAddress << 1, 0xEA, 1, dataBytes, 1, HAL_MAX_DELAY);
    HAL_Delay(100);

    // Read 4-byte device serial number
    HAL_I2C_Mem_Read(&hi2c1, lidarliteAddress << 1, 0x16, 1, dataBytes, 4, HAL_MAX_DELAY);

    // Append the desired I2C address to the end of the serial number byte array
    dataBytes[4] = newAddress;

    // Write the serial number and new address in one 5-byte transaction
    HAL_I2C_Mem_Write(&hi2c1, lidarliteAddress << 1, 0x16, 1, dataBytes, 5, HAL_MAX_DELAY);

    // Wait for the I2C peripheral to be restarted with new device address
    HAL_Delay(100);

    // If desired, disable default I2C device address (using the new I2C device address)
    if (disableDefault)
    {
        dataBytes[0] = 0x01; // set bit to disable default address
        HAL_I2C_Mem_Write(&hi2c1, newAddress << 1, 0x1b, 1, dataBytes, 1, HAL_MAX_DELAY);

        // Wait for the I2C peripheral to be restarted with new device address
        HAL_Delay(100);
    }

    // Disable flash storage
    dataBytes[0] = 0;
    HAL_I2C_Mem_Write(&hi2c1, newAddress << 1, 0xEA, 1, dataBytes, 1, HAL_MAX_DELAY);
    HAL_Delay(100);
    CheckDevice(newAddress);
}

uint16_t GetDistance(uint8_t lidarAddr){
	uint32_t startTick = HAL_GetTick(); // Get current tick for timeout

	// 1. Write 0x04 to register 0x00.
	HAL_I2C_Mem_Write(&hi2c1, lidarAddr << 1, ACQ_COMMAND, 1, &TAKE_DIST, 1, HAL_MAX_DELAY);


	do { // 2. Read register 0x01.
	  HAL_I2C_Mem_Read(&hi2c1, lidarAddr << 1, STATUS_REG, 1, &status, 1, HAL_MAX_DELAY);

	  if((HAL_GetTick() - startTick) > TIMEOUT_MS) {// Handle timeout
	  	    return 0;  // return invalid distance value
	  }

	} while (status & 0x01); // 3. Repeat step 2 until bit 0 (LSB) goes low.


	// 4. Read two bytes from 0x10 (low byte 0x10 then high byte 0x11) to obtain the 16-bit measured distance in centimeters.
	HAL_I2C_Mem_Read(&hi2c1, lidarAddr << 1, DISTANCE_REG_LOW, 1, &distance_low, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, lidarAddr << 1, DISTANCE_REG_HIGH, 1, &distance_high, 1, HAL_MAX_DELAY);

	return (((uint16_t)distance_high << 8) | distance_low);
}

void CheckRightSensor() {
	while(CheckDevice(LIDAR_ADDR1) != HAL_OK) {
		  sprintf(msg, "device 1\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
	  }
	  R_RED_LED();
	  HAL_Delay(100);
	  R_OFF_LED();
	  HAL_Delay(100);
	  R_RED_LED();
	  HAL_Delay(100);
	  R_OFF_LED();
	  HAL_Delay(100);
}

void CheckLeftSensor() {
	while(CheckDevice(LIDAR_ADDR2) != HAL_OK) {
		  sprintf(msg, "device 2\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
	  }
	  L_RED_LED();
	  HAL_Delay(100);
	  L_OFF_LED();
	  HAL_Delay(100);
	  L_RED_LED();
	  HAL_Delay(100);
	  L_OFF_LED();
	  HAL_Delay(100);
}

#endif /* SRC_GARMIN_LIDAR_LITEV4_H_ */
