/**
  ******************************************************************************
  * @file    bmp180.h
  * @brief   This file contains all the constants parameters for the BMP180
  * @Date	 23 Jul 2021
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BMP180_H
#define BMP180_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Driver --------------------------------------------------------------------*/
//#define LL_Driver

#ifdef LL_Driver
#define I2Cx						I2C1
#else
#define I2Cx						hi2c1
#endif

/* BMP180 Register -----------------------------------------------------------*/
#define BMP180_I2C_ADDRESS			0x77
#define BMP180_REG_ChipID			0xD0
#define BMP180_REG_SoftReset		0xE0
#define BMP180_REG_Calibration		0xAA
#define BMP180_REG_MEASURE_CTRL		0XF4
#define BMP180_REG_LSB				0xF6

#define BMP180_CMD_SoftReset		0xB6
#define BMP180_CMD_UTemp			0x2E
#define BMP180_CMD_UPress			0x34

#define BMP180_ChipID				0x55

#define atmPress 					101000 // Pressure at Sea Level
#define Alt							35.5

/* BMP180 Data Structure -----------------------------------------------------*/
typedef struct {
	short			AC1;
	short			AC2;
	short			AC3;
	unsigned short	AC4;
	unsigned short	AC5;
	unsigned short	AC6;
	short			B1;
	short		 	B2;
    long			B3;
    unsigned long	B4;
    long			B5;
    long			B6;
    unsigned long	B7;
    short 			MB;
    short 			MC;
    short 			MD;
    long			X1;
    long			X2;
    long			X3;
    long			UT;
    long			UP;
    long			P;
    long			T;
} calib_t;

typedef struct {
	float pressure;
	float temperature;
	float altitude;
	float sealevel;
} data_t;

typedef struct {
	uint8_t id;
	uint8_t oss;
	calib_t calibs;
    data_t 	data;
} BMP180_drv_t;

/* BMP180 External Function --------------------------------------------------*/
void BMP180_Delay(uint16_t Delay);
uint8_t BMP180_Init(BMP180_drv_t *dev);
uint8_t BMP180_GetTemp(BMP180_drv_t *dev, calib_t *Cal);
uint8_t BMP180_GetPress(BMP180_drv_t *dev, calib_t *Cal);
void BMP180_GetEnviroment(BMP180_drv_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* BMP180_H */
