/*---------------------------------------------------------------------------/
/ SC8815 General Firmware Library Header File V0.1 (C)Sghz, 2021
/------------------------------------------------- ---------------------------/
/The SC8815 library is free software developed in accordance with the license policy under the following conditions.
/
/ Copyright (C) 2021, Sghz, all rights reserved.
/
/ 1. Redistribution of the source code must retain the above copyright notice, this condition and the following disclaimer.
/
/ This software is provided "as is" by the copyright owner and contributors, and any guarantees related to this software are not liable.
/ The copyright owner or contributor is not responsible for any damage caused by the use of this software.
/---------------------------------------------------------------------------*/


//Recursive including protection
#ifndef SC8815_H
#define SC8815_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdint.h>


//To use this library, you need to provide the following external functions, which are the functions that the SC8815 library needs to use
extern void I2C_WriteRegByte(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t ByteData);   //Write a byte to the device register via I2C
extern uint8_t I2C_ReadRegByte(uint8_t SlaveAddress, uint8_t RegAddress);                   //Read a byte from the device register via I2C
extern void SoftwareDelay(uint8_t ms);                                                      //Software delay in milliseconds

//Set the sensing resistance value on the power path of SC8815, which should be an integer like 10 or 5 (unit: mOhm)
#define SCHW_VBUS_RSHUNT        10          //Sense resistor value on VBUS current path
#define SCHW_BATT_RSHUNT        10          //Sensing resistance value on the battery current path

//Set the voltage divider resistor value on the FB pin. When the VBUS feedback mode used is external feedback, the library uses these values to calculate the corresponding output voltage (unit: Ohm)
#define SCHW_FB_RUP             105000      //RUP is the resistance value connected from FB to VBUS
#define SCHW_FB_RDOWM           33000       //RDOWM is the resistance value connected from FB to GND

//Function status definition
#define sENABLE 0x01 //Enable function
#define sDISABLE 0x00 //Disable function

//SC8815 register address definition
#define SC8815_ADDR             0xE8 //SC8815 I2C device address
#define SCREG_VBAT_SET          0x00 //Battery configuration register
#define SCREG_VBUSREF_I_SET     0x01 //VBUS internal reference register 1
#define SCREG_VBUSREF_I_SET2    0x02 //VBUS internal reference register 2
#define SCREG_VBUSREF_E_SET     0x03 //VBUS external reference register 1
#define SCREG_VBUSREF_E_SET2    0x04 //VBUS external reference register 2
#define SCREG_IBUS_LIM_SET      0x05 //VBUS current limit setting register
#define SCREG_IBAT_LIM_SET      0x06 //Battery current limit setting register
#define SCREG_VINREG_SET        0x07 //VINREG setting register
#define SCREG_RATIO             0x08 //Ratio setting register
#define SCREG_CTRL0_SET         0x09 //Hardware configuration register 0
#define SCREG_CTRL1_SET         0x0A //Hardware configuration register 1
#define SCREG_CTRL2_SET         0x0B //Hardware configuration register 2
#define SCREG_CTRL3_SET         0x0C //Hardware configuration register 3
#define SCREG_VBUS_FB_VALUE     0x0D //VBUS voltage feedback register 1
#define SCREG_VBUS_FB_VALUE2    0x0E //VBUS voltage feedback register 2
#define SCREG_VBAT_FB_VALUE     0x0F //VBAT voltage feedback register 1
#define SCREG_VBAT_FB_VALUE2    0x10 //VBAT voltage feedback register 2
#define SCREG_IBUS_VALUE        0x11 //BUS current feedback register 1
#define SCREG_IBUS_VALUE2       0x12 //BUS current feedback register 2
#define SCREG_IBAT_VALUE        0x13 //BAT current feedback register 1
#define SCREG_IBAT_VALUE2       0x14 //BAT current feedback register 2
#define SCREG_ADIN_VALUE        0x15 //ADIN pin voltage feedback register 1
#define SCREG_ADIN_VALUE2       0x16 //ADIN pin voltage feedback register 2
#define SCREG_STATUS            0x17 //Interrupt status register
#define SCREG_MASK              0x19 //Interrupt mask register


//SC8815 Battery configuration structure
typedef struct
{
    uint8_t IRCOMP; /*Battery internal resistance compensation setting.
                                     -This parameter should take the value defined in SCBAT_IRCOMP*/

    uint8_t VBAT_SEL; /*Battery voltage feedback mode setting.
                                     -This parameter should take the value defined in SCBAT_VSEL*/

    uint8_t CSEL; /*Set the number of battery cells in series.
                                     -This parameter should take the value defined in SCBAT_CSEL*/

    uint8_t VCELL; /*Set the voltage of a single battery.
                                     -This parameter should take the value defined in SCBAT_VCELL*/

}SC8815_BatteryConfigTypeDef;

//SC8815 hardware initialization structure
typedef struct
{
    uint8_t IBAT_RATIO; /*Battery current ratio setting.
                                    -This parameter should take the value defined in SCHWI_IBAT_RATIO*/

    uint8_t IBUS_RATIO; /*VBUS current ratio setting.
                                    -This parameter should take the value defined in SCHWI_IBUS_RATIO*/

    uint8_t VBAT_RATIO; /*Battery voltage ratio setting.
                                    -This parameter should take the value defined in SCHWI_VBAT_RATIO*/

    uint8_t VBUS_RATIO; /*VBUS voltage ratio setting.
                                    -This parameter should take the value defined in SCHWI_VBUS_RATIO*/

    uint8_t VINREG_Ratio; /*VINREG voltage ratio setting.
                                    -This parameter should take the value defined in SCHWI_VINREG_RATIO*/

    uint8_t SW_FREQ; /*Set the switching frequency of SC8815.
                                    -This parameter should take the value defined in SCHWI_FREQ*/

    uint8_t DeadTime; /*Set the switch dead time.
                                    -This parameter should take the value defined in SCHWI_DT*/

    uint8_t ICHAR; /*Set the charging current reference, the trickle charging current and termination current will be based on this reference.
                                    -This parameter should take the value defined in SCHWI_ICHAR*/

    uint8_t TRICKLE; /*Set the off and on of the trickle charging function.
                                    -This parameter should take the value defined in SCHWI_TRICKLE*/

    uint8_t TERM; /*Turn on or turn off the automatic charging termination function.
                                    -This parameter should take the value defined in SCHWI_TERM*/

    uint8_t FB_Mode; /*Set the VBUS voltage feedback mode of SC8815 reverse discharge mode.
                                    -This parameter should take the value defined in SCHWI_FB_MODE*/

    uint8_t TRICKLE_SET; /*Set the trickle charge phase gate value.
                                    -This parameter should take the value defined in SCHWI_TRICKLE_SET*/

    uint8_t OVP; /*Turn on or turn off the overvoltage protection function in the reverse discharge mode of SC8815.
                                    -This parameter should take the value defined in SCHWI_OVP*/

    uint8_t DITHER; /*Turn on or turn off the frequency dithering function of SC8815.
                                    -This parameter should take the value defined in SCHWI_DITHER*/

    uint8_t SLEW_SET; /*Set the rate of VBUS dynamic change in SC8815 reverse discharge mode.
                                    -This parameter should take the value defined in SCHWI_SLEW_SET*/

    uint8_t ADC_SCAN; /*Turn on or off the internal ADC conversion function of SC8815.
                                    -This parameter should take the value defined in SCHWI_ADC_SCAN*/

    uint8_t ILIM_BW; /*Set the bandwidth of the SC8815 current-limiting loop.
                                    -This parameter should take the value defined in SCHWI_ILIM_BW*/

    uint8_t LOOP; /*Set SC8815 loop response control mode.
                                    -This parameter should take the value defined in SCHWI_LOOP*/

    uint8_t ShortFoldBack; /*Turn on or turn off the VBUS short-circuit protection function in the reverse discharge mode of SC8815.
                                    -This parameter should take the value defined in SCHWI_SFB*/

    uint8_t EOC; /*Set the detection gate value of SC8815 charging end.
                                    -This parameter should take the value defined in SCHWI_EOC*/

    uint8_t PFM; /*Turn on or off the PFM mode of SC8815 under light load during reverse discharge.
                                    -This parameter should take the value defined in SCHWI_PFM*/

}SC8815_HardwareInitTypeDef;

//SC8815 Interrupt state structure
typedef struct
{
    uint8_t AC_OK; /*The adapter insertion interrupt is detected at the ACIN pin.*/

     uint8_t INDET; /*The adapter insertion interrupt is detected at the INDET pin.*/

     uint8_t VBUS_SHORT; /*VBUS short circuit fault interrupt detected in reverse discharge mode.*/

     uint8_t OTP; /* OTP (over temperature) fault interrupt occurred.*/

     uint8_t EOC; /*Satisfy EOC (end of charge) condition interrupt.*/

     /* When used as an interrupt configuration, the member is set to sENABLE(1) to indicate that the interrupt is turned on, and sDISABLE(0) is set to indicate that the interrupt is turned off
     * When used to obtain interrupt status, a member value of 1 means that an interrupt has occurred, and 0 means that an interrupt has not occurred.
     * The interrupt status will be cleared after reading the interrupt status register.*/

}SC8815_InterruptStatusTypeDef;


/*
* SC8815 Battery configuration structure parameters
* @{
*/
   // SCBAT_IRCOMP parameter definition
    #define SCBAT_IRCOMP_0mR 0x00 //Battery internal resistance is 0 mOhm
    #define SCBAT_IRCOMP_20mR 0x40 //Battery internal resistance is 20 mOhm
    #define SCBAT_IRCOMP_40mR 0x80 //Battery internal resistance is 40 mOhm
    #define SCBAT_IRCOMP_80mR 0xC0 //The internal resistance of the battery is 80 mOhm

    // SCBAT_VSEL parameter definition
    #define SCBAT_VBAT_SEL_Internal 0x00 //Battery voltage feedback is internally set (VBATS pin is directly connected to the battery)
    #define SCBAT_VBAT_SEL_External 0x20 //External settings (The battery voltage is connected to the VBATS pin through a voltage divider resistor)

    // SCBAT_CSEL parameter definition
    #define SCBAT_CSEL_1S 0x00 //The number of battery cells is 1
    #define SCBAT_CSEL_2S 0x08 //The number of battery cells is 2
    #define SCBAT_CSEL_3S 0x10 //The number of battery cells is 3
    #define SCBAT_CSEL_4S 0x18 //The number of battery cells is 4

    // SCBAT_VCELL parameter definition
    #define SCBAT_VCELL_4v10 0x00 //The voltage of a single battery is 4.10V
    #define SCBAT_VCELL_4v20 0x01 //The voltage of a single battery is 4.20V
    #define SCBAT_VCELL_4v25 0x02 //The voltage of a single battery is 4.25V
    #define SCBAT_VCELL_4v30 0x03 //The voltage of a single battery is 4.30V
    #define SCBAT_VCELL_4v35 0x04 //The voltage of a single battery is 4.35V
    #define SCBAT_VCELL_4v40 0x05 //The voltage of a single battery is 4.40V
    #define SCBAT_VCELL_4v45 0x06 //The voltage of a single battery is 4.45V
    #define SCBAT_VCELL_4v50 0x07 //The voltage of a single battery is 4.50V
/*
* @}
*/

/*
* SC8815 Hardware initialization structure parameters
* @{
*/
   //* SCHWI_IBAT_RATIO parameter definition
    #define SCHWI_IBAT_RATIO_6x 0x00 //The battery current ratio is 6x (the maximum current limit is 6A when the sensing resistance is 10mOhm)
    #define SCHWI_IBAT_RATIO_12x 0x10 //The battery current ratio is 12x (the maximum current limit is 12A when the sensing resistance is 10mOhm)

    // SCHWI_IBUS_RATIO parameter definition
    #define SCHWI_IBUS_RATIO_6x 0x04 //VBUS current ratio is 6x (the maximum current limit is 6A when the sensing resistance is 10mOhm)
    #define SCHWI_IBUS_RATIO_3x 0x08 //VBUS current ratio is 3x (the maximum current limit is 3A when the sensing resistance is 10mOhm)

    // SCHWI_VBAT_RATIO parameter definition
    #define SCHWI_VBAT_RATIO_12_5x 0x00 //The battery voltage ratio is 12.5x, which only affects the SC8815 built-in ADC measurement range, the maximum measurement value is 25.6V (applicable to more than 2 battery applications)
    #define SCHWI_VBAT_RATIO_5x 0x02 //The battery voltage ratio is 5x, which only affects the measurement range of the built-in ADC of SC8815, the maximum measurement value is 10.24V (applicable to battery applications with less than 3 cells)

    // SCHWI_VBUS_RATIO parameter definition
    #define SCHWI_VBUS_RATIO_12_5x 0x00 //VBUS voltage ratio is 12.5x, which affects the built-in ADC measurement range and OTG reverse output voltage range, the maximum operating value is 25.6V, step 25mV
    #define SCHWI_VBUS_RATIO_5x 0x01 //VBUS voltage ratio is 5x, which affects the built-in ADC measurement range and OTG reverse output voltage range, the maximum operating value is 10.24V, step 10mV

    // SCHWI_VINREG_RATIO parameter definition
    #define SCHWI_VINREG_RATIO_100x 0x00 //VINREG ratio is 100x (100mV step adjustment, the maximum value is 25.6V)
    #define SCHWI_VINREG_RATIO_40x 0x10 //VINREG ratio is 40x (40mV step adjustment, the maximum value is 10.24V)

    // SCHWI_FREQ parameter definition
    #define SCHWI_FREQ_150KHz 0x00 //The switching frequency is 150KHz (boost and buck modes are both 150KHz)
    #define SCHWI_FREQ_300KHz_1 0x04 //The switching frequency is 300KHz (300KHz in boost mode, 150KHz in buck mode)
    #define SCHWI_FREQ_300KHz_2 0x08 //The switching frequency is 300KHz (boost and buck modes are both 300KHz)
    #define SCHWI_FREQ_450KHz 0x0C //The switching frequency is 450KHz (boost and buck modes are 450KHz)

    // SCHWI_DT parameter definition
    #define SCHWI_DT_20ns 0x00 //The dead time is 20ns
    #define SCHWI_DT_40ns 0x01 //The dead time is 40ns
    #define SCHWI_DT_60ns 0x02 //The dead time is 60ns
    #define SCHWI_DT_80ns 0x03 //The dead time is 80ns

   // SCHWI_ICHAR parameter definition
    #define SCHWI_ICHAR_IBUS 0x00 //Select VBUS current as reference
    #define SCHWI_ICHAR_IBAT 0x80 //Select battery (IBAT) current as reference

    // SCHWI_TRICKLE parameter definition
    #define SCHWI_TRICKLE_Enable 0x00 //Turn on trickle charging
    #define SCHWI_TRICKLE_Disable 0x40 //Turn off trickle charging

    // SCHWI_TERM parameter definition
    #define SCHWI_TERM_Enable 0x00 //Turn on and automatically terminate the charging
    #define SCHWI_TERM_Disable 0x20 //Turn off charging and automatically terminate

    // SCHWI_FB_MODE parameter definition
    #define SCHWI_FB_Internal 0x00 //OTG reverse discharge mode VBUS voltage feedback uses VBUS_PIN to feedback inside the chip
    #define SCHWI_FB_External 0x10 //OTG reverse discharge mode VBUS voltage feedback uses external divider resistor network feedback on FB_PIN

    // SCHWI_TRICKLE_SET parameter definition
    #define SCHWI_TRICKLE_SET_70 0x00 //The trickle charge phase gate value is 70% of the battery voltage setting
    #define SCHWI_TRICKLE_SET_60 0x08 //The trickle charge phase gate value is 60% of the battery voltage setting

    // SCHWI_OVP parameter definition
    #define SCHWI_OVP_Enable 0x00 //Enable VBUS overvoltage protection in SC8815 reverse discharge mode
    #define SCHWI_OVP_Disable 0x04 //Disable VBUS overvoltage protection in SC8815 reverse discharge mode

    // SCHWI_DITHER parameter definition
    #define SCHWI_DITHER_Disable 0x00 //Disable the SC8815 frequency jitter function, the PGATE pin is used for PMOS control
    #define SCHWI_DITHER_Enable 0x04 //Enable the SC8815 frequency jitter function, the PGATE pin is used to set the frequency jitter

    // SCHWI_SLEW_SET parameter definition
    #define SCHWI_SLEW_1mV_us 0x00 //VBUS dynamic change rate in reverse discharge mode = 1mV/us
    #define SCHWI_SLEW_2mV_us 0x01 //VBUS dynamic change rate in reverse discharge mode = 2mV/us
    #define SCHWI_SLEW_4mV_us 0x02 //VBUS dynamic change rate in reverse discharge mode = 4mV/us
    #define SCHWI_SLEW_8mV_us 0x03 //VBUS dynamic change rate in reverse discharge mode = 8mV/us

    // SCHWI_ADC_SCAN parameter definition
    #define SCHWI_ADC_Disable 0x00 //Disable the SC8815 ADC conversion to save 1-2mA of power consumption
    #define SCHWI_ADC_Enable 0x20 //Open the SC8815 ADC conversion, at this time you can read the voltage and current data

    // SCHWI_ILIM_BW parameter definition
    #define SCHWI_ILIM_BW_5KHz 0x00 //SC8815 current limit loop bandwidth is 5KHz
    #define SCHWI_ILIM_BW_1_25KHz 0x10 //SC8815 current limit loop bandwidth is 1.25KHz
    
    // SCHWI_LOOP parameter definition
    #define SCHWI_LOOP_Normal 0x00 //SC8815 loop response mode is normal loop response
    #define SCHWI_LOOP_Improved 0x08 //SC8815 loop response mode is to improve loop response

    // SCHWI_SFB parameter definition
    #define SCHWI_SFB_Enable 0x00 //Enable VBUS and BATT short circuit protection in SC8815 reverse discharge mode
    #define SCHWI_SFB_Disable 0x04 //Disable VBUS and BATT short circuit protection in SC8815 reverse discharge mode

    // SCHWI_EOC parameter definition
    #define SCHWI_EOC_1_25 0x00 //The detection threshold of SC8815 charging end is 1/25 of the charging current setting
    #define SCHWI_EOC_1_10 0x02 //The detection threshold of SC8815 charging end is 1/10 of the charging current setting

    // SCHWI_PFM parameter definition
    #define SCHWI_PFM_Disable 0x00 //Disable the PFM mode of SC8815 under light load during reverse discharge
    #define SCHWI_PFM_Enable 0x01 //Enable the PFM mode of SC8815 under light load during reverse discharge
/*
* @}
*/

//Example Demo of initializing SC8815
void SC8815_Init_Demo(void);

//SC8815 Hardware configuration initialization function
void SC8815_BatteryConfig(SC8815_BatteryConfigTypeDef *SC8815_BatteryConfigStruct);     //Configure SC8815 battery parameters (need to be configured with SC8815 PSTOP pin high)
void SC8815_HardwareInit(SC8815_HardwareInitTypeDef *SC8815_HardwareInitStruct);        //Initialize the SC8815 hardware configuration (need to be configured with SC8815 PSTOP pin high)
void SC8815_ConfigInterruptMask(SC8815_InterruptStatusTypeDef *InterruptStatusStruct);  // Configure SC8815 interrupt mask (interrupt enable or disable)

//SC8815 Read interrupt status function
void SC8815_ReadInterrupStatus(SC8815_InterruptStatusTypeDef *InterruptStatusStruct);

//SC8815 reads the built-in ADC conversion result function
uint16_t SC8815_Read_VBUS_Voltage(void); //Read VBUS voltage
uint16_t SC8815_Read_VBUS_Current(void); //Read VBUS current
uint16_t SC8815_Read_BATT_Voltage(void); //Read battery voltage
uint16_t SC8815_Read_BATT_Current(void); //Read battery current
uint16_t SC8815_Read_ADIN_Voltage(void); //Read ADIN_PIN voltage

//SC8815 set parameter value function
void SC8815_SetOutputVoltage(uint16_t NewVolt);     //Set OTG reverse output voltage
void SC8815_SetBusCurrentLimit(uint16_t NewILIM);   //Set VBUS current limit
void SC8815_SetBatteryCurrLimit(uint16_t NewILIM);  //Set battery current limit
void SC8815_VINREG_SetVoltage(uint16_t NewVolt);    //Set VINREG voltage

//SC8815 Get parameter setting value function
uint16_t SC8815_GetOutputVoltage(void);     //Get OTG reverse output voltage
uint16_t SC8815_GetBusCurrentLimit(void);   //Get VBUS current limit
uint16_t SC8815_GetBatteryCurrLimit(void);  //Get battery current limit
uint16_t SC8815_VINREG_GetVoltage(void);    //Get VINREG voltage

//SC8815 set hardware configuration function (no need for SC8815 PSTOP pin to be high)
void SC8815_OTG_Enable(void);               //Open OTG reverse discharge mode
void SC8815_OTG_Disable(void);              //Close OTG reverse discharge mode, SC8815 returns to charging mode
void SC8815_VINREG_SetRatio_40x(void);      //Set the gain of VINREG to 40x
void SC8815_VINREG_SetRatio_100x(void);     //Set the gain of VINREG to 100x
void SC8815_OVP_Enable(void);               //Turn on the VBUS overvoltage protection function in OTG mode
void SC8815_OVP_Disable(void);              //Disable the VBUS overvoltage protection function in OTG mode
void SC8815_PGATE_Enable(void);             //Enable PGATE pin function, output low level to turn on PMOS
void SC8815_PGATE_Disable(void);            //Disable PGATE pin function, output high level to close PMOS
void SC8815_GPO_Enable(void);               //Enable GPO pin function, output low level
void SC8815_GPO_Disable(void);              //Disable GPO pin function, output high impedance state
void SC8815_ADC_Enable(void);               //Enable ADC scan, at this time ADC data can be read
void SC8815_ADC_Disable(void);              //Turn off ADC scanning to save 1-2mA of power consumption
void SC8815_SFB_Enable(void);               //Open the VBUS and VBAT short circuit protection function in OTG mode
void SC8815_SFB_Disable(void);              //Disable VBUS and VBAT short circuit protection function in OTG mode
void SC8815_PFM_Enable(void);               //Open PFM mode under light load condition in OTG mode
void SC8815_PFM_Disable(void);              //Disable PFM mode under light load condition in OTG mode

//SC8815 Get hardware configuration status function
uint8_t SC8815_OTG_IsEnable(void);          //Check if OTG is on
uint8_t SC8815_VINREG_GetRatio(void);       //Get the gain of VINREG
uint8_t SC8815_OVP_IsEnable(void);          //Check whether the OVP function is on in OTG mode
uint8_t SC8815_PGATE_IsEnable(void);        //Check whether the PGATE pin function is on
uint8_t SC8815_GPO_IsEnable(void);          //Check whether the GPO pin function is on
uint8_t SC8815_ADC_IsEnable(void);          //Check whether the ADC scan is on
uint8_t SC8815_SFB_IsEnable(void);          //Check whether the short circuit protection function in OTG mode is on
uint8_t SC8815_PFM_IsEnable(void);          //Check whether the PFM mode in OTG mode is on

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // !SC8815_H

/*****END OF FILE****/
