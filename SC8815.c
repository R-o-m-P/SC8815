/*---------------------------------------------------------------------------/
/  SC8815 library V0.1 (C)Sghz，2021
/----------------------------------------------------------------------------/
/ SC8815The library is free software developed in accordance with the license policy under the following conditions.
/
/Copyright (C) 2021, Sghz, all rights reserved.
/
/ 1. Redistribution of the source code must retain the above copyright notice, this condition and the following disclaimer.
/
/ This software is provided "as is" by the copyright owner and contributors, and any guarantees related to this software are not liable.
/ The copyright owner or contributor is not responsible for any damage caused by the use of this software.
/---------------------------------------------------------------------------*/

//Include header file
#include "SC8815.h"

/****************************************
* @brief    Example Demo function to initialize SC8815
*****************************************/
void SC8815_Init_Demo(void)
{
    SC8815_BatteryConfigTypeDef SC8815_BatteryConfigStruct = { 0 };
    SC8815_HardwareInitTypeDef SC8815_HardwareInitStruct = { 0 };
    SC8815_InterruptStatusTypeDef SC8815_InterruptMaskInitStruct = { 0 };

    /****starting SC8815...****/
    //->Set PSTOP to high
     //->Set CE to low
     //SoftwareDelay(5); //Necessary start delay

     //Configure SC8815 battery parameter options
    SC8815_BatteryConfigStruct.IRCOMP = SCBAT_IRCOMP_20mR;
    SC8815_BatteryConfigStruct.VBAT_SEL = SCBAT_VBAT_SEL_Internal;
    SC8815_BatteryConfigStruct.CSEL = SCBAT_CSEL_4S;
    SC8815_BatteryConfigStruct.VCELL = SCBAT_VCELL_4v25;
    SC8815_BatteryConfig(&SC8815_BatteryConfigStruct);

   //Configuring SC8815 hardware parameter options
    SC8815_HardwareInitStruct.IBAT_RATIO = SCHWI_IBAT_RATIO_6x;
    SC8815_HardwareInitStruct.IBUS_RATIO = SCHWI_IBUS_RATIO_3x;
    SC8815_HardwareInitStruct.VBAT_RATIO = SCHWI_VBAT_RATIO_12_5x;
    SC8815_HardwareInitStruct.VBUS_RATIO = SCHWI_VBUS_RATIO_12_5x;
    SC8815_HardwareInitStruct.VINREG_Ratio = SCHWI_VINREG_RATIO_100x;
    SC8815_HardwareInitStruct.SW_FREQ = SCHWI_FREQ_300KHz_2;
    SC8815_HardwareInitStruct.DeadTime = SCHWI_DT_40ns;
    SC8815_HardwareInitStruct.ICHAR = SCHWI_ICHAR_IBAT;
    SC8815_HardwareInitStruct.TRICKLE = SCHWI_TRICKLE_Enable;
    SC8815_HardwareInitStruct.TERM = SCHWI_TERM_Enable;
    SC8815_HardwareInitStruct.FB_Mode = SCHWI_FB_Internal;
    SC8815_HardwareInitStruct.TRICKLE_SET = SCHWI_TRICKLE_SET_70;
    SC8815_HardwareInitStruct.OVP = SCHWI_OVP_Enable;
    SC8815_HardwareInitStruct.DITHER = SCHWI_DITHER_Disable;
    SC8815_HardwareInitStruct.SLEW_SET = SCHWI_SLEW_1mV_us;
    SC8815_HardwareInitStruct.ADC_SCAN = SCHWI_ADC_Disable;
    SC8815_HardwareInitStruct.ILIM_BW = SCHWI_ILIM_BW_1_25KHz;
    SC8815_HardwareInitStruct.LOOP = SCHWI_LOOP_Normal;
    SC8815_HardwareInitStruct.ShortFoldBack = SCHWI_SFB_Enable;
    SC8815_HardwareInitStruct.EOC = SCHWI_EOC_1_25;
    SC8815_HardwareInitStruct.PFM = SCHWI_PFM_Disable;
    SC8815_HardwareInit(&SC8815_HardwareInitStruct);

   //Configuring SC8815 interrupt mask option
    SC8815_InterruptMaskInitStruct.AC_OK = sENABLE;
    SC8815_InterruptMaskInitStruct.INDET = sENABLE;
    SC8815_InterruptMaskInitStruct.VBUS_SHORT = sENABLE;
    SC8815_InterruptMaskInitStruct.OTP = sENABLE;
    SC8815_InterruptMaskInitStruct.EOC = sENABLE;
    SC8815_ConfigInterruptMask(&SC8815_InterruptMaskInitStruct);
    /***Now you can set the PSTOP pin to low to start SC8815 power conversion****/



   /*** Example 1, set to charge mode, battery and VBUS current limit 2A, VINREG set to 12V ****/
    SC8815_SetBatteryCurrLimit(2000);
    SC8815_SetBusCurrentLimit(2000);
    SC8815_VINREG_SetVoltage(12000);
    SC8815_OTG_Disable();


    /*** Example 2, set to reverse discharge mode, battery and VBUS current limit 3A, output voltage set to 12V ****/
    //SC8815_SetBatteryCurrLimit(2000);
    //SC8815_SetBusCurrentLimit(2000);
    //SC8815_SetOutputVoltage(12000);
    //SC8815_OTG_Enable();


    /*** Example 3, read SC8815 ADC data ****/
    //uint16_t VbusVolt = SC8815_Read_VBUS_Voltage();
    //uint16_t VbusCurr = SC8815_Read_VBUS_Current();
    //uint16_t BattVolt = SC8815_Read_BATT_Voltage();
    //uint16_t BattCurr = SC8815_Read_BATT_Current();


   /*** Example 4, read SC8815 interrupt status ****/
    SC8815_ReadInterrupStatus(&SC8815_InterruptMaskInitStruct);     //MCU calls this function after receiving the SC8815 interrupt to read the SC8815 interrupt (after reading the interrupt status, the interrupt status bit will be cleared)
    if (SC8815_InterruptMaskInitStruct.AC_OK == 1)                  //Check if the AC_OK interrupt is triggered
    {
        // AC_OK interrupt handling code
    }
    else if (SC8815_InterruptMaskInitStruct.EOC == 1)
    {
        // EOC interrupt handling code
    }
}

/****************************************
* @brief Configuring SC8815 battery parameters
* @param SC8815_BatteryConfigStruct Pointer to SC8815 battery configuration structure
* @note Before configuring, start SC8815 and delay 5ms to ensure that the device is ready. You also need SC8815 PSTOP pin to be high to perform this hardware configuration
*****************************************/
void SC8815_BatteryConfig(SC8815_BatteryConfigTypeDef *SC8815_BatteryConfigStruct)
{
    uint8_t tmp;
    tmp = SC8815_BatteryConfigStruct->IRCOMP;
    tmp |= SC8815_BatteryConfigStruct->VBAT_SEL;
    tmp |= SC8815_BatteryConfigStruct->CSEL;
    tmp |= SC8815_BatteryConfigStruct->VCELL;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_VBAT_SET, tmp);
}
/****************************************
* @brief Initializing SC8815 hardware configuration
* @param SC8815_HardwareInitStruct Pointer to SC8815 hardware initialization structure
* @note Before configuring, start SC8815 and delay 5ms to ensure that the device is ready. You also need SC8815 PSTOP pin to be high to perform this hardware configuration
*****************************************/
void SC8815_HardwareInit(SC8815_HardwareInitTypeDef *SC8815_HardwareInitStruct)
{
    uint8_t tmp;

    //Ratio configuration
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0xE0; //Read the reserved bits in the register (these reserved bits cannot be moved)
    tmp |= SC8815_HardwareInitStruct->IBAT_RATIO;           //Loading configuration parameters
    tmp |= SC8815_HardwareInitStruct->IBUS_RATIO;
    tmp |= SC8815_HardwareInitStruct->VBAT_RATIO;
    tmp |= SC8815_HardwareInitStruct->VBUS_RATIO;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_RATIO, tmp);    //Write back to register

    //Hardware configuration 0
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x60;
    tmp |= SC8815_HardwareInitStruct->VINREG_Ratio;
    tmp |= SC8815_HardwareInitStruct->SW_FREQ;
    tmp |= SC8815_HardwareInitStruct->DeadTime;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL0_SET, tmp);

    //Hardware configuration 1
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) & 0x03;
    tmp |= SC8815_HardwareInitStruct->ICHAR;
    tmp |= SC8815_HardwareInitStruct->TRICKLE;
    tmp |= SC8815_HardwareInitStruct->TERM;
    tmp |= SC8815_HardwareInitStruct->FB_Mode;
    tmp |= SC8815_HardwareInitStruct->TRICKLE_SET;
    tmp |= SC8815_HardwareInitStruct->OVP;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL1_SET, tmp);

    //Hardware configuration 2
    tmp = (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL2_SET) & 0xF0) | 0x08;
    tmp |= SC8815_HardwareInitStruct->DITHER;
    tmp |= SC8815_HardwareInitStruct->SLEW_SET;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL2_SET, tmp);

    //Hardware configuration 3 (no reserved bits)
    tmp = SC8815_HardwareInitStruct->ADC_SCAN;
    tmp |= SC8815_HardwareInitStruct->ILIM_BW;
    tmp |= SC8815_HardwareInitStruct->LOOP;
    tmp |= SC8815_HardwareInitStruct->ShortFoldBack;
    tmp |= SC8815_HardwareInitStruct->EOC;
    tmp |= SC8815_HardwareInitStruct->PFM;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, tmp);

    //Delay required after the configuration is complete
    SoftwareDelay(5);
}
/****************************************
* @brief Configure SC8815 interrupt mask (interrupt enable or disable)
* @param InterruptStatusStruct Pointer to SC8815 interrupt status configuration structure
*****************************************/
void SC8815_ConfigInterruptMask(SC8815_InterruptStatusTypeDef *InterruptStatusStruct)
{
    uint8_t tmp;
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_MASK) & 0x91;
    tmp |= InterruptStatusStruct->AC_OK ? 0x40 : 0x00;
    tmp |= InterruptStatusStruct->INDET ? 0x20 : 0x00;
    tmp |= InterruptStatusStruct->VBUS_SHORT ? 0x08 : 0x00;
    tmp |= InterruptStatusStruct->OTP ? 0x04 : 0x00;
    tmp |= InterruptStatusStruct->EOC ? 0x02 : 0x00;
    I2C_WriteRegByte(SC8815_ADDR, SCREG_MASK, tmp);
}

/****************************************
* @brief read the interrupt status of SC8815
* @param InterruptStatusStruct Pointer to SC8815 interrupt status structure
*****************************************/
void SC8815_ReadInterrupStatus(SC8815_InterruptStatusTypeDef *InterruptStatusStruct)
{
    uint8_t tmp;
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_MASK);     //Read status register
    InterruptStatusStruct->AC_OK = (tmp & 0x40) >> 6;   //Disassemble every bit
    InterruptStatusStruct->INDET = (tmp & 0x20) >> 5;
    InterruptStatusStruct->VBUS_SHORT = (tmp & 0x08) >> 3;
    InterruptStatusStruct->OTP = (tmp & 0x04) >> 2;
    InterruptStatusStruct->EOC = (tmp & 0x02) >> 1;
}

/****************************************
* @brief reads the measurement result of the VBUS voltage by the built-in ADC of SC8815
* @return VBUS voltage in mV
*****************************************/
uint16_t SC8815_Read_VBUS_Voltage(void)
{
    uint8_t tmp1, tmp2;
    double RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x01) == 1) ? 5 : 12.5; //Get the ratio of VBUS voltage
    tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUS_FB_VALUE);           //Read ADC value register 1
    tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUS_FB_VALUE2) >> 6;     //Read ADC value register 2

    //Return VBUS voltage value
    return (uint16_t)((4 * tmp1 + tmp2 + 1) * RATIO_Value) * 2;
}
/****************************************
* @brief reads the measurement result of the VBUS current by the built-in ADC of SC8815
* @return VBUS current in mA (SC8815 has no current direction distinction)
*****************************************/
uint16_t SC8815_Read_VBUS_Current(void)
{
    uint8_t tmp1, tmp2;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x0C) == 4) ? 6 : 3;    //IBUS ratio
    tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBUS_VALUE);              //Read ADC value register 1
    tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBUS_VALUE2) >> 6;        //Read ADC value register 2

    //Return VBUS current value
    return ((uint16_t)(50 * RATIO_Value) * (4 * tmp1 + tmp2 + 1)) / (3 * SCHW_VBUS_RSHUNT);
}
/****************************************
* @brief reads the measurement result of the battery voltage by the built-in ADC of SC8815
* @return battery voltage in mV
*****************************************/
uint16_t SC8815_Read_BATT_Voltage(void)
{
    uint8_t tmp1, tmp2;
    double RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x02) == 2) ? 5 : 12.5; //Get the ratio of battery voltage
    tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBAT_FB_VALUE);           //Read ADC value register 1
    tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBAT_FB_VALUE2) >> 6;     //Read ADC value register 2
    //Return battery voltage value
    return (uint16_t)((4 * tmp1 + tmp2 + 1) * RATIO_Value) * 2;
}
/****************************************
* @brief reads the measurement result of the battery current measured by the built-in ADC of the SC8815
* @return battery current in mA (SC8815 has no current direction distinction)
*****************************************/
uint16_t SC8815_Read_BATT_Current(void)
{
    uint8_t tmp1, tmp2;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x10) == 16) ? 12 : 6;  //Get the IBAT ratio
    tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBAT_VALUE);              //Read ADC value register 1
    tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBAT_VALUE2) >> 6;        //Read ADC value register 2

    //Return IBAT current value
    return (uint16_t)((50 * RATIO_Value) * (4 * tmp1 + tmp2 + 1)) / (3 * SCHW_BATT_RSHUNT);
}
/****************************************
* @brief reads the measurement result of the ADIN_PIN voltage by the built-in ADC of SC8815
* @return ADIN_PIN voltage in mV
*****************************************/
uint16_t SC8815_Read_ADIN_Voltage(void)
{
    uint8_t tmp1,tmp2;

    tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_ADIN_VALUE);          //Read ADC value register 1
    tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_ADIN_VALUE2) >> 6;    //Read ADC value register 2
    //Return ADIN_PIN voltage value
    return (uint16_t)(4 * tmp1 + tmp2 + 1) * 2;
}

/****************************************
* @brief sets the output voltage of SC8815 when OTG output is enabled
* @param NewVolt The new value in mV of the output voltage 
* @note Do not input a voltage value that exceeds the maximum settable value. For example, the maximum output is 1024mV, inputting 1145mV will cause the calculation result to overflow error
*****************************************/
void SC8815_SetOutputVoltage(uint16_t NewVolt)
{
    uint16_t tmp1,tmp2;
    double RATIO_Value;

    //Determine the mode of VBUS voltage feedback
    if (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) & 0x10)
    {
        RATIO_Value = (double)SCHW_FB_RUP / SCHW_FB_RDOWM + 1.0;    //Calculate the output voltage ratio
        tmp1 = (NewVolt / RATIO_Value) / 2;                         //Calculate the corresponding reference voltage
        //Get the value of VBUSREF register 2
        for (tmp2 = 0; tmp2 < 3; tmp2++)
        {
            if (((tmp1 - tmp2 - 1) % 4) == 0)
            {
                break;
            }
        }

        //Get the value of VBUSREF register 1
        tmp1 = (tmp1 - tmp2 - 1) / 4;

        //Write to SC8815 VBUSREF_E_SET register
        I2C_WriteRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET, (uint8_t)tmp1);
        I2C_WriteRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET2, (uint8_t)tmp2);
    }
    else
    {
        RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x01) == 1) ? 5 : 12.5; //取得 VBUS 电压的比率
        tmp1 = NewVolt / RATIO_Value / 2;   //Calculate the corresponding reference voltage
        //Get the value of VBUSREF register 2
        for (tmp2 = 0; tmp2 < 3; tmp2++)
        {
            if (((tmp1 - tmp2 - 1) % 4) == 0)
            {
                break;
            }
        }

        //Get the value of VBUSREF register 1
        tmp1 = (tmp1 - tmp2 - 1) / 4;

        //Write to SC8815 VBUSREF_I_SET register
        I2C_WriteRegByte(SC8815_ADDR, SCREG_VBUSREF_I_SET, (uint8_t)tmp1);
        I2C_WriteRegByte(SC8815_ADDR, SCREG_VBUSREF_I_SET2, (uint8_t)tmp2);
    }
}
/****************************************
* @brief sets the current limit value on the SC8815 VBUS path, common in both directions
* @param NewILIM The new unit is mA output current limit setting value
* @note The minimum current limit value should not be lower than 300mA, and the current value that exceeds the maximum settable value must not be input
*****************************************/
void SC8815_SetBusCurrentLimit(uint16_t NewILIM)
{
    uint8_t tmp;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x0C) == 4) ? 6 : 3;    //Rate of obtaining IBUS
    tmp = (16 * (NewILIM) * (SCHW_VBUS_RSHUNT)) / (625 * RATIO_Value) - 1;              //Calculate LIM settings
    I2C_WriteRegByte(SC8815_ADDR, SCREG_IBUS_LIM_SET, tmp);                             //Write to SC8815 register
}
/****************************************
* @brief sets the current limit value on the battery path of SC8815, universal in both directions
* @param NewILIM The new battery current limit set value in mA
* @note The minimum current limit value should not be lower than 300mA, and the current value that exceeds the maximum settable value must not be input
*****************************************/
void SC8815_SetBatteryCurrLimit(uint16_t NewILIM)
{
    uint8_t tmp;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x10) == 16) ? 12 : 6; //Get IBAT ratio
    tmp = (16 * (NewILIM) * (SCHW_VBUS_RSHUNT)) / (625 * RATIO_Value) - 1;             //Calculate LIM settings
    I2C_WriteRegByte(SC8815_ADDR, SCREG_IBAT_LIM_SET, tmp);                            //Write to SC8815 register
}
/****************************************
* @brief set SC8815 VINREG voltage value (similar to MPPT)
* @param NewVolt The new unit is mV VINREG voltage setting value
* @note must not enter a voltage value that exceeds the maximum settable value
*****************************************/
void SC8815_VINREG_SetVoltage(uint16_t NewVolt)
{
    uint16_t RATIO_Value;
    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x10) == 16) ? 40 : 100;    //Get the ratio of VINREG
    I2C_WriteRegByte(SC8815_ADDR, SCREG_VINREG_SET, (NewVolt / RATIO_Value) - 1);               //Write to SC8815 register
}

/****************************************
* @brief gets the output voltage setting value of SC8815 in OTG reverse output
* @return The output voltage setting value in mV
*****************************************/
uint16_t SC8815_GetOutputVoltage(void)
{
    uint16_t tmp1, tmp2;
    double RATIO_Value;

    //Determine the mode of VBUS voltage feedback
    if (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) & 0x10)
    {
        //Read the VBUSREF_E_SET register
        tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET);
        tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET2);

        //Calculate the output voltage ratio
        RATIO_Value = (double)SCHW_FB_RUP / SCHW_FB_RDOWM + 1.0;
    }
    else
    {
        //Read the VBUSREF_E_SET register
        tmp1 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET);
        tmp2 = I2C_ReadRegByte(SC8815_ADDR, SCREG_VBUSREF_E_SET2);

        //Get the ratio of VBUS voltage
        RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x01) == 1) ? 5 : 12.5;
    }

    //Return the corresponding output voltage
    return (uint16_t)((4 * tmp1 + tmp2 + 1) * RATIO_Value) * 2;
}
/****************************************
* @brief gets the current limit setting value on the SC8815 VBUS path
* @return VBUS path current limit setting value in mA
*****************************************/
uint16_t SC8815_GetBusCurrentLimit(void)
{
    uint8_t tmp;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x0C) == 4) ? 6 : 3; //Get IBUS ratio
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBUS_LIM_SET);                          //Get the IBUS current limit register value
    //Return IBUS current limit value
    return (uint16_t)((uint32_t)(625 * (RATIO_Value) * (tmp + 1)) / (16 * (SCHW_VBUS_RSHUNT)));
}
/****************************************
* @brief Get the current limit setting value on the battery path of SC8815
* @return The battery path current limit setting value in mA
*****************************************/
uint16_t SC8815_GetBatteryCurrLimit(void)
{
    uint8_t tmp;
    uint16_t RATIO_Value;

    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_RATIO) & 0x10) == 16) ? 12 : 6; //Get IBAT ratio
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_IBAT_LIM_SET);                            //Get the IBAT current limit register value

    //Return IBAT current limit value
    return (uint16_t)((uint32_t)(625 * (RATIO_Value) * (tmp + 1)) / (16 * (SCHW_BATT_RSHUNT)));
}
/****************************************
* @brief Get SC8815 VINREG voltage setting value
* @return VINREG voltage setting value in mV
*****************************************/
uint16_t SC8815_VINREG_GetVoltage(void)
{
    uint8_t tmp;
    uint16_t RATIO_Value;
    RATIO_Value = ((I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x10) == 16) ? 40 : 100; //Get VINREG ratio
    tmp = I2C_ReadRegByte(SC8815_ADDR, SCREG_VINREG_SET);                                    //Get the VINREG register value
    return tmp * RATIO_Value;
}

/****************************************
* @brief   Turn on OTG reverse discharge mode
*****************************************/
void SC8815_OTG_Enable(void)
{
    //Set EN_OTG bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL0_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) | 0x80);
}
/****************************************
* @brief    Turn off OTG reverse discharge mode, SC8815 will be in charging mode
*****************************************/
void SC8815_OTG_Disable(void)
{
    //Clear EN_OTG bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL0_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x7F);
}
/****************************************
* @brief    Set the gain of VINREG to 40x
*****************************************/
void SC8815_VINREG_SetRatio_40x(void)
{
    //Set VINREG_RATIO bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL0_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) | 0x10);
}
/****************************************
* @brief   Set the gain of VINREG to 100x
*****************************************/
void SC8815_VINREG_SetRatio_100x(void)
{
    //Clear VINREG_RATIO bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL0_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0xEF);
}
/****************************************
* @brief    Turn on the VBUS overvoltage protection function in OTG reverse discharge mode
*****************************************/
void SC8815_OVP_Enable(void)
{
    //Clear DIS_OVP bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL1_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) & 0xFB);
}
/****************************************
* @brief    Turn off the VBUS overvoltage protection function in OTG reverse discharge mode
*****************************************/
void SC8815_OVP_Disable(void)
{
    //Set DIS_OVP bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL1_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) | 0x04);
}
/****************************************
* @brief   Turn on PGATE pin function, output low level to turn on PMOS
*****************************************/
void SC8815_PGATE_Enable(void)
{
    //Set EN_PGATE bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) | 0x80);
}
/****************************************
* @brief   Turn off PGATE pin function, output high level to turn off PMOS
*****************************************/
void SC8815_PGATE_Disable(void)
{
    //Clear the EN_PGATE bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x7F);
}
/****************************************
* @brief    Open GPO pin function, output low level
*****************************************/
void SC8815_GPO_Enable(void)
{
    //Set the GPO_CTRL bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) | 0x40);
}
/****************************************
* @brief    Disable GPO pin function, output high impedance state
*****************************************/
void SC8815_GPO_Disable(void)
{
    //Clear the GPO_CTRL bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0xBF);
}
/****************************************
* @brief    Turn on ADC scan, you can read ADC data at this time
*****************************************/
void SC8815_ADC_Enable(void)
{
    //Set AD_START bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) | 0x20);
}
/****************************************
* @brief    Turn off ADC scanning, saving 1-2mA of power consumption
*****************************************/
void SC8815_ADC_Disable(void)
{
    //Clear AD_START bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0xDF);
}
/****************************************
* @brief    Turn on the VBUS and VBAT short-circuit protection function in OTG reverse discharge mode
*****************************************/
void SC8815_SFB_Enable(void)
{
    //Clear the DIS_ShortFoldBack bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0xFB);
}
/****************************************
* @brief    Disable the VBUS and VBAT short-circuit protection function in OTG reverse discharge mode
*****************************************/
void SC8815_SFB_Disable(void)
{
    //Set DIS_ShortFoldBack bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) | 0x04);
}
/****************************************
* @brief    Enable PFM mode under light load conditions in OTG mode
*****************************************/
void SC8815_PFM_Enable(void)
{
    //Set the EN_PFM bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) | 0x01);
}
/****************************************
* @brief    Turn off PFM mode under light load conditions in OTG mode
*****************************************/
void SC8815_PFM_Disable(void)
{
    //Clear the EN_PFM bit
    I2C_WriteRegByte(SC8815_ADDR, SCREG_CTRL3_SET, I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0xFE);
}

/****************************************
* @brief check if OTG is on
* @return OTG function status (1b or 0b)
*****************************************/
uint8_t SC8815_OTG_IsEnable(void)
{
    //Return OTG status
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x80) ? 1 : 0;
}
/****************************************
* @brief gets the gain of VINREG
* @return VINREG gain (40 or 100)
*****************************************/
uint8_t SC8815_VINREG_GetRatio(void)
{
    //Return the gain of VINREG
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL0_SET) & 0x10) ? 40 : 100;
}
/****************************************
* @brief Check whether the OVP (overvoltage protection) function in OTG mode is on
* @return OVP function status (1b or 0b)
*****************************************/
uint8_t SC8815_OVP_IsEnable(void)
{
    //Return the status of OVP
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL1_SET) & 0x04) ? 0 : 1;
}
/****************************************
* @brief check whether the PGATE pin function is on
* @return PGATE pin function status (1b or 0b)
*****************************************/
uint8_t SC8815_PGATE_IsEnable(void)
{
    //Return the status of PGATE
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x80) ? 1 : 0;
}
/****************************************
* @brief check whether the GPO pin function is on
* @return GPO pin function status (1b or 0b)
*****************************************/
uint8_t SC8815_GPO_IsEnable(void)
{
    //Returns the status of the GPO
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x40) ? 1 : 0;
}
/****************************************
* @brief check if ADC scan is on
* @return ADC scan status (1b or 0b)
*****************************************/
uint8_t SC8815_ADC_IsEnable(void)
{
    //Return the status of the ADC
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x20) ? 1 : 0;
}
/****************************************
* @brief Check whether the short-circuit protection function is on in OTG mode
* @return The status of the short-circuit protection function (1b or 0b)
*****************************************/
uint8_t SC8815_SFB_IsEnable(void)
{
    //Return to the state of the short-circuit protection function
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x04) ? 0 : 1;
}
/****************************************
* @brief check if the PFM mode in OTG mode is on
* @return PFM mode status (1b or 0b)
*****************************************/
uint8_t SC8815_PFM_IsEnable(void)
{
    //Return the status of PFM
    return (I2C_ReadRegByte(SC8815_ADDR, SCREG_CTRL3_SET) & 0x01) ? 1 : 0;
}

/*****END OF FILE****/
