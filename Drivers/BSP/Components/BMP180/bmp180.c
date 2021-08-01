/**
  ******************************************************************************
  * @file    bmp180.c
  * @brief   This file includes the HAL/LL driver for BMP180 I2C sensor
  ******************************************************************************
  */
#include "bmp180.h"

/**
  * @brief  The function is used as delay
  * @param  Delay	Target delay in ms
  */
void BMP180_Delay(uint16_t Delay)
{
#ifdef LL_Driver
	LL_mDelay(Delay);
#else
	HAL_Delay(Delay);
#endif
}

/**
  * @brief  I2C Bus Read
  * @retval Success = 0, Failed = 1
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be read
  */
uint8_t BMP180_Bus_Read(uint8_t DevAddr, uint8_t MemAddr, uint8_t *pData,
		uint16_t Len)
{
#ifdef LL_Driver
    uint16_t XferCount = Len;
    uint16_t XferSize = 0;

	/* Wait for I2C bus is free */
	while (LL_I2C_IsActiveFlag_BUSY(I2Cx)) {};

	if (!LL_I2C_IsEnabled(I2Cx))
	{
		LL_I2C_Enable(I2Cx);
	}

    /* Send Slave Address and Memory Address */
    LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT, 1,
    		LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {};

	 /* Send Memory Address */
    LL_I2C_TransmitData8(I2Cx, ((uint8_t)(uint16_t)(MemAddr & (uint16_t)0x00FF)));
    while (!LL_I2C_IsActiveFlag_TC(I2Cx)) {};

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    if (XferCount > 255U)
    {
        XferSize = 255U;
        LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_RELOAD, LL_I2C_GENERATE_START_READ);
    }
    else
    {
        XferSize = XferCount;
        LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_AUTOEND, LL_I2C_GENERATE_START_READ);
    }

    do
    {
        /* Wait until RXNE flag is set */
        while (!LL_I2C_IsActiveFlag_RXNE(I2Cx)) {};

        /* Read data from RXDR */
        *pData = LL_I2C_ReceiveData8(I2Cx);

        /* Increment Buffer pointer */
        pData++;
        XferCount--;
        XferSize--;

        if ((XferCount != 0U) && (XferSize == 0U))
        {
            /* Wait until TCR flag is set */
            while (!LL_I2C_IsActiveFlag_TCR(I2Cx)) {};

            if (XferCount > 255U)
            {
                XferSize = 255U;

                LL_I2C_HandleTransfer(I2Cx, DevAddr,
                		LL_I2C_ADDRSLAVE_7BIT, (uint8_t)XferSize, I2C_CR2_RELOAD,
						LL_I2C_GENERATE_NOSTARTSTOP);
            }
            else
            {
                XferSize = XferCount;
                LL_I2C_HandleTransfer(I2Cx, DevAddr,
                		LL_I2C_ADDRSLAVE_7BIT, (uint8_t)XferSize, I2C_CR2_AUTOEND,
						LL_I2C_GENERATE_NOSTARTSTOP);
            }
        }
    } while (XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically
     * generated.
     * Wait until STOPF flag is reset */
    while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {};

	/* Clear NACKF Flag */
    LL_I2C_ClearFlag_NACK(I2Cx);

    /* Clear STOP Flag */
    LL_I2C_ClearFlag_STOP(I2Cx);

    /* Clear Configuration Register 2 */
    I2Cx->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R |
    		I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));
#else
	if(HAL_I2C_Mem_Read(&I2Cx, DevAddr, MemAddr, 1, pData, Len,
			HAL_MAX_DELAY) != 0)
	{
		return 1;
	}
#endif

	return 0;
}

/**
  * @brief  I2C Bus Write
  * @retval Success = 0, Failed = 1
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be Write
  */
static uint8_t BMP180_Bus_Write(uint8_t DevAddr, uint8_t MemAddr,
		uint8_t *pData, uint16_t Len)
{
#ifdef LL_Driver
	uint16_t XferCount = Len;
	uint8_t *TxBuffer = pData;
	uint16_t XferSize = 0;

	/* Wait for I2C bus is free */
	while (LL_I2C_IsActiveFlag_BUSY(I2Cx)) {};

	if (!LL_I2C_IsEnabled(I2Cx))
	{
		LL_I2C_Enable(I2Cx);
	}

    /* Send Slave Address and Memory Address */
    LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT, 1,
    		I2C_CR2_RELOAD, LL_I2C_GENERATE_START_WRITE);

    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)){};

	 /* Send Memory Address */
    LL_I2C_TransmitData8(I2Cx, ((uint8_t)(uint16_t)(MemAddr & (uint16_t)0x00FF)));
	while (!LL_I2C_IsActiveFlag_TCR(I2Cx)) {};

    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    if (XferCount > 255U)
    {
        XferSize = 255U;
        LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_RELOAD , LL_I2C_GENERATE_NOSTARTSTOP);
    }
    else
    {
        XferSize = XferCount;
        LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_AUTOEND , LL_I2C_GENERATE_NOSTARTSTOP);
    }

    do
    {
        /* Wait until TXIS flag is set */
    	while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {};

        /* Write data to TXDR */
        LL_I2C_TransmitData8(I2Cx, *TxBuffer);

        /* Increment Buffer pointer */
        TxBuffer++;

        XferCount--;
        XferSize--;

        if ((XferCount != 0U) && (XferSize == 0U))
        {
            /* Wait until TCR flag is set */
        	while (!LL_I2C_IsActiveFlag_TCR(I2Cx)) {};

            if (XferCount > 255U)
            {
                XferSize = 255U;
                LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
                		(uint8_t)XferSize, I2C_CR2_RELOAD ,
						LL_I2C_GENERATE_NOSTARTSTOP);
            }
            else
            {
                XferSize = XferCount;
                LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
                		(uint8_t)XferSize, I2C_CR2_AUTOEND ,
						LL_I2C_GENERATE_NOSTARTSTOP);
            }
        }
    } while (XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically
     * generated
     * Wait until STOPF flag is reset */
    while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {};

    /* Clear NACKF Flag */
    LL_I2C_ClearFlag_NACK(I2Cx);

    /* Clear STOP Flag */
    LL_I2C_ClearFlag_STOP(I2Cx);

    /* Clear Configuration Register 2 */
    I2Cx->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R |
    		I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));
#else
	if(HAL_I2C_Mem_Write(&I2Cx, DevAddr, MemAddr, 1, pData, Len,
			HAL_MAX_DELAY) != 0)
	{
		return 1;
	}
#endif

	return 0;
}

/**
  * @brief  This internal function read chip id from register and store it in
  * 		device structure
  */
static uint8_t BMP180_Read_ChipID(BMP180_drv_t *dev)
{
	if(BMP180_Bus_Read((uint8_t)(BMP180_I2C_ADDRESS << 1), BMP180_REG_ChipID,
			&dev->id, 1) != 0)
	{
		return 1;
	}
	return 0;
}

/**
  * @brief  This internal function reads the calibration data from the sensor,
  * 		parse it and store in the device structure
  */
static uint8_t BMP180_Read_Calibration(BMP180_drv_t *dev)
{
	uint8_t rxbuff[22] = {0};
	if(BMP180_Bus_Read((uint8_t)(BMP180_I2C_ADDRESS << 1),
			BMP180_REG_Calibration, rxbuff, sizeof(rxbuff)) != 0)
	{
		return 1;
	}

	dev->calibs.AC1 = ((rxbuff[0] << 8) | rxbuff[1]);
	dev->calibs.AC2 = ((rxbuff[2] << 8) | rxbuff[3]);
	dev->calibs.AC3 = ((rxbuff[4] << 8) | rxbuff[5]);
	dev->calibs.AC4 = ((rxbuff[6] << 8) | rxbuff[7]);
	dev->calibs.AC5 = ((rxbuff[8] << 8) | rxbuff[9]);
	dev->calibs.AC6 = ((rxbuff[10] << 8) | rxbuff[11]);
	dev->calibs.B1 = ((rxbuff[12] << 8) | rxbuff[13]);
	dev->calibs.B2 = ((rxbuff[14] << 8) | rxbuff[15]);
	dev->calibs.MB = ((rxbuff[16] << 8) | rxbuff[17]);
	dev->calibs.MC = ((rxbuff[18] << 8) | rxbuff[19]);
	dev->calibs.MD = ((rxbuff[20] << 8) | rxbuff[21]);

	return 0;
}

/**
  * @brief  This function called to initiate sensor with all parameter
  */
uint8_t BMP180_Init(BMP180_drv_t *dev)
{
	/* read Chip Id */
	if(BMP180_Read_ChipID(dev) != 0)
	{
		return 1;
	}

	/* Check for the correct chip id */
	if(dev->id != BMP180_ChipID)
	{
		return 1;
	}

	/* Read the calibration data */
	if(BMP180_Read_Calibration(dev) != 0)
	{
		return 1;
	}

	return 0;
}

/**
  * @brief  This internal function called to get sensor raw temperature
  */
static uint8_t BMP180_Get_UTemp(BMP180_drv_t *dev)
{
	uint8_t CMD = BMP180_CMD_UTemp;
	if(BMP180_Bus_Write((uint8_t)(BMP180_I2C_ADDRESS << 1),
			BMP180_REG_MEASURE_CTRL, &CMD, 1) != 0)
	{
		return 1;
	}

	BMP180_Delay(5);	// Wait 4.5ms
	uint8_t UTemp[2];
	if(BMP180_Bus_Read((uint8_t)(BMP180_I2C_ADDRESS << 1), BMP180_REG_LSB,
			UTemp, 2) != 0)
	{
		return 1;
	}

	dev->calibs.UT = (UTemp[0]<<8) + UTemp[1];
	return 0;
}

/**
  * @brief  This internal function called to get sensor raw pressure
  */
static uint8_t BMP180_Get_UPress(BMP180_drv_t *dev)
{
	uint8_t CMD = BMP180_CMD_UPress + (dev->oss << 6);
	if(BMP180_Bus_Write((uint8_t)(BMP180_I2C_ADDRESS << 1),
			BMP180_REG_MEASURE_CTRL, &CMD, 1) != 0)
	{
		return 1;
	}

	switch (dev->oss)
	{
		case (0):
			BMP180_Delay(5);
			break;
		case (1):
			BMP180_Delay(8);
			break;
		case (2):
			BMP180_Delay(14);
			break;
		case (3):
			BMP180_Delay(26);
			break;
	}

	uint8_t UPress[3] = {0,};
	if(BMP180_Bus_Read((uint8_t)(BMP180_I2C_ADDRESS << 1), BMP180_REG_LSB,
			UPress, 3) != 0)
	{
		return 1;
	}

	dev->calibs.UP = (((UPress[0]<<16) + (UPress[1]<<8) +
			UPress[2]) >> (8 - (dev->oss)));

	return 0;
}

/**
  * @brief  This function is used to called the raw temperature data and
  * 		return the compensated temperature data in integer data type
  */
uint8_t BMP180_GetTemp(BMP180_drv_t *dev, calib_t *Cal)
{
	if(BMP180_Get_UTemp(dev) != 0)
	{
		return 1;
	}

	Cal->X1 = ((Cal->UT - Cal->AC6) * (Cal->AC5 / (pow(2, 15))));
	Cal->X2 = ((Cal->MC * (pow(2, 11))) / (Cal->X1 + Cal->MD));
	Cal->B5 = Cal->X1 + Cal->X2;
	Cal->T = ((Cal->B5 + 8) / pow(2, 4));

	return 0;
}

/**
  * @brief  This function is used to called the raw pressure data and
  * 		return the compensated pressure data in integer data type
  */
uint8_t BMP180_GetPress(BMP180_drv_t *dev, calib_t *Cal)
{
	if(BMP180_Get_UPress(dev) != 0)
	{
		return 1;
	}

	Cal->X1 = ((Cal->UT - Cal->AC6) * (Cal->AC5 / (pow(2,15))));
	Cal->X2 = ((Cal->MC * (pow(2,11))) / (Cal->X1 + Cal->MD));
	Cal->B5 = Cal->X1 + Cal->X2;
	Cal->B6 = Cal->B5 - 4000;
	Cal->X1 = (Cal->B2 * (Cal->B6 * Cal->B6 / (pow(2,12)))) / (pow(2,11));
	Cal->X2 = Cal->AC2 * Cal->B6 / (pow(2,11));
	Cal->X3 = Cal->X1 + Cal->X2;
	Cal->B3 = (((Cal->AC1 * 4 + Cal->X3) << dev->oss) + 2) / 4;
	Cal->X1 = Cal->AC3 * Cal->B6 / pow(2,13);
	Cal->X2 = (Cal->B1 * (Cal->B6 * Cal->B6 / (pow(2,12)))) / (pow(2,16));
	Cal->X3 = ((Cal->X1 + Cal->X2) + 2) / pow(2,2);
	Cal->B4 = Cal->AC4 * (unsigned long)(Cal->X3 + 32768) / (pow(2,15));
	Cal->B7 = ((unsigned long)Cal->UP - Cal->B3)*(50000 >> dev->oss);
	if (Cal->B7 < 0x80000000)
	{
		Cal->P = (Cal->B7 * 2) / Cal->B4;
	}else{
		Cal->P = (Cal->B7 / Cal->B4) * 2;
	}

	Cal->X1 = (Cal->P / (pow(2,8))) * (Cal->P / (pow(2,8)));
	Cal->X1 = (Cal->X1 * 3038) / (pow(2,16));
	Cal->X2 = (-7357 * Cal->P) / (pow(2,16));
	Cal->P = Cal->P + (Cal->X1 + Cal->X2 + 3791) / (pow(2,4));

	return 0;
}

/**
  * @brief  This function is used to called the altitude and return the altitude
  * 		data in float data type
  */
static void BMP180_GetAltitude(BMP180_drv_t *dev)
{
	dev->data.altitude = 44330 * (1 - (pow((dev->calibs.P/(float)atmPress),
			0.19029495718)));
}

/**
  * @brief  This function is used to calculate the sea level pressure base on
  * 		altitude 0(sea level) and return pressure data in float type
  */
void BMP180_GetSeaLevel(BMP180_drv_t *dev)
{
	dev->data.sealevel = dev->calibs.P / pow((1 - (Alt / 44330)), 5.255);
}

/**
  * @brief  This function is used to called to get all environment data and
  * 		return in data structure
  */
void BMP180_GetEnviroment(BMP180_drv_t *dev)
{
	BMP180_GetTemp(dev, &dev->calibs);
	BMP180_GetPress(dev, &dev->calibs);
	BMP180_GetAltitude(dev);
	BMP180_GetSeaLevel(dev);
	dev->data.temperature = dev->calibs.T / 10.0;
	dev->data.pressure = dev->calibs.P / 100.0;
}


