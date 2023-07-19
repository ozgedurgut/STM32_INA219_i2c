#include "main.h"
#include "INA219_2.h"

uint16_t Read16_2(INA219_t_2 *ina219, uint8_t Register)
{
	uint8_t Value[2];

	HAL_I2C_Mem_Read(ina219->ina219_i2c_2, (INA219_ADDRESS_2<<1), Register, 1, Value, 2, 1000);

	return ((Value[0] << 8) | Value[1]);
}


void Write16_2(INA219_t_2 *ina219, uint8_t Register, uint16_t Value)
{
	uint8_t addr[2];
	addr[0] = (Value >> 8) & 0xff;  // upper byte
	addr[1] = (Value >> 0) & 0xff; // lower byte
	HAL_I2C_Mem_Write(ina219->ina219_i2c_2, (INA219_ADDRESS_2<<1), Register, 1, (uint8_t*)addr, 2, 1000);
}

uint16_t INA219_ReadBusVoltage_2(INA219_t_2 *ina219)
{
	uint16_t result = Read16_2(ina219, INA219_REG_BUSVOLTAGE);

	return ((result >> 3  ) * 4);

}

int16_t INA219_ReadCurrent_raw_2(INA219_t_2 *ina219)
{
	int16_t result = Read16_2(ina219, INA219_REG_CURRENT);

	return (result );
}

float INA219_ReadCurrent_2(INA219_t_2 *ina219)
{
	int16_t result_raw = INA219_ReadCurrent_raw_2(ina219);

	return result_raw * ina219_currentDivider_mA_2/100.f;
}

uint16_t INA219_ReadShuntVolage_2(INA219_t_2 *ina219)
{
	uint16_t result = Read16_2(ina219, INA219_REG_SHUNTVOLTAGE);

	return (result * 0.01 );
}

void INA219_Reset_2(INA219_t_2 *ina219)
{
	Write16_2(ina219, INA219_REG_CONFIG, INA219_CONFIG_RESET);
	HAL_Delay(1);
}

void INA219_setCalibration_2(INA219_t_2 *ina219, uint16_t CalibrationData)
{
	Write16_2(ina219, INA219_REG_CALIBRATION, CalibrationData);
}

uint16_t INA219_getConfig_2(INA219_t_2 *ina219)
{
	uint16_t result = Read16_2(ina219, INA219_REG_CONFIG);
	return result;
}

void INA219_setConfig_2(INA219_t_2 *ina219, uint16_t Config)
{
	Write16_2(ina219, INA219_REG_CONFIG, Config);
}

void INA219_setCalibration_32V_2A_2(INA219_t_2 *ina219)
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	             INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
	             INA219_CONFIG_SADCRES_12BIT_1S_532US |
	             INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219_calibrationValue_2 = 4096;
	ina219_currentDivider_mA_2 = 10; // Current LSB = 100uA per bit (1000/100 = 10)
	ina219_powerMultiplier_mW_2 = 2; // Power LSB = 1mW per bit (2/1)

	INA219_setCalibration_2(ina219, ina219_calibrationValue_2);
	INA219_setConfig_2(ina219, config);
}

void INA219_setCalibration_32V_1A_2(INA219_t_2 *ina219)
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219_calibrationValue_2 = 10240;
	ina219_currentDivider_mA_2 = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
	ina219_powerMultiplier_mW_2 = 0.8f; // Power LSB = 800uW per bit

	INA219_setCalibration_2(ina219, ina219_calibrationValue_2);
	INA219_setConfig_2(ina219, config);
}

void INA219_setCalibration_16V_400mA_2(INA219_t_2 *ina219)
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
	                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219_calibrationValue_2 = 8192;
	ina219_currentDivider_mA_2 = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
	ina219_powerMultiplier_mW_2 = 1.0f; // Power LSB = 1mW per bit

	INA219_setCalibration_2(ina219, ina219_calibrationValue_2);
	INA219_setConfig_2(ina219, config);
}

void INA219_setPowerMode_2(INA219_t_2 *ina219, uint8_t Mode)
{
	uint16_t config = INA219_getConfig_2(ina219);

	switch (Mode) {
		case INA219_CONFIG_MODE_POWERDOWN:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_POWERDOWN & INA219_CONFIG_MODE_MASK);
			INA219_setConfig_2(ina219, config);
			break;

		case INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED & INA219_CONFIG_MODE_MASK);
			INA219_setConfig_2(ina219, config);
			break;

		case INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS & INA219_CONFIG_MODE_MASK);
			INA219_setConfig_2(ina219, config);
			break;

		case INA219_CONFIG_MODE_ADCOFF:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_ADCOFF & INA219_CONFIG_MODE_MASK);
			INA219_setConfig_2(ina219, config);
			break;
	}
}

uint8_t INA219_Init_2(INA219_t_2 *ina219, I2C_HandleTypeDef *i2c, uint8_t Address_2)
{
	ina219->ina219_i2c_2 = i2c;
	ina219->Address_2 = Address_2;

	ina219_currentDivider_mA_2 = 0;
	ina219_powerMultiplier_mW_2 = 0;

	uint8_t ina219_isReady = HAL_I2C_IsDeviceReady(i2c, (Address_2 << 1), 3, 2);

	if(ina219_isReady == HAL_OK)
	{

		INA219_Reset_2(ina219);
		INA219_setCalibration_32V_2A_2(ina219);

		return 1;
	}

	else
	{
		return 0;
	}
}
