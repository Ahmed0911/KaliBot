/*
 * ADCDrv.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: User
 */

#include "ADCDrv.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

ADCDrv::ADCDrv()
{
	// defaults, in case EEPROM load fails
	CalibCSA = 2048;
	CalibCSB = 2048;
	CalibCSC = 2048;
}

void ADCDrv::Init()
{
	memset(m_Values, 0, sizeof(m_Values));

	// ADC
	// PE0 - AIN3 - Current Sense A
	// PE1 - AIN2 - Current Sense B
	// PE2 - AIN1 - Current Sense C
	// PK1 - AIN17 - Voltage Phase A
	// PK2 - AIN18 - Voltage Phase B
	// PK3 - AIN19 - Voltage Phase C
	// PK0 - AIN16 - PVDD Voltage
	// PE3 - AIN0 - Temperature
	// PD6 - AIN5 - Pot Voltage

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);
	GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 ); // PK0, PK1, PK2, PK3
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 ); // PE0, PE1, PE2, PE3
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_6 ); // PD6
	

	// configure sequencers, main
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PWM0 | ADC_TRIGGER_PWM_MOD0, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH3); // PK1 - AIN3 - CS-A
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH2); // PK2 - AIN2 - CS-B
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH1); // PK3 - AIN1 - CS-C
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH17); // PK1 - AIN17 - VPH-A
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH18); // PK2 - AIN18 - VPH-B
	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH19); // PK3 - AIN19 - VPH-C
	ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH16); // PK0 - AIN16 - PVDD
	ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END); // PE3 - AIN0 - Temperature
	ADCSequenceEnable(ADC0_BASE, 0);

	// Aux sequence 0 use only one INT from sequence 0, but triggered every time
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PWM0 | ADC_TRIGGER_PWM_MOD0, 1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH5); // PD6 - AIN5 - Pot Voltage
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_TS | ADC_CTL_END); // Int. Temp
	ADCSequenceEnable(ADC0_BASE, 1);

	// enable INT (main only)
	PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_TR_CNT_LOAD);
	ADCIntClear(ADC0_BASE, 0);
	ADCIntEnable(ADC0_BASE, 0);
	IntPrioritySet(INT_ADC0SS0, (0x01 << 5)); // 0x00 - highest priority
	IntEnable(INT_ADC0SS0);
}

void ADCDrv::IntHandler()
{
	unsigned int vals[8];
	int rd = ADCSequenceDataGet(ADC0_BASE, 0, vals);
	if( rd == 8 )
	{
		memcpy(m_Values, vals, sizeof(m_Values));
	}

	// Get Aux data
	rd = ADCSequenceDataGet(ADC0_BASE, 1, vals);
	if( rd == 2 )
	{
		memcpy(m_AuxValues, vals, sizeof(m_AuxValues));
	}
}

unsigned int ADCDrv::GetValue(int channel)
{
	return m_Values[channel];
}

float ADCDrv::CSM_A()
{
	float delta = (float)m_Values[ADCCSM_A]-CalibCSA;
	float adcVoltage = 3 * delta / 4096.0;
	float current = adcVoltage * 25; // 0.002 V/A * 20 gain => 0.04 V/A => 25A/V

	return current;
}

float ADCDrv::CSM_B()
{
	float delta = (float)m_Values[ADCCSM_B]-CalibCSB;
	float adcVoltage = 3 * delta / 4096.0;
	float current = adcVoltage * 25; // 0.002 V/A * 20 gain => 0.04 V/A => 25A/V

	return current;
}

float ADCDrv::CSM_C()
{
	float delta = (float)m_Values[ADCCSM_C]-CalibCSC;
	float adcVoltage = 3 * delta / 4096.0;
	float current = adcVoltage * 25; // 0.002 V/A * 20 gain => 0.04 V/A => 25A/V

	return current;
}

float ADCDrv::VPHASE_A()
{
	float voltage = (3.0f * m_Values[ADCVPHASE_A] / 4096.0f) * 9.2546f; // [V]

	return voltage;
}

float ADCDrv::VPHASE_B()
{
	float voltage = (3.0f * m_Values[ADCVPHASE_B] / 4096.0f) * 9.2546f; // [V]

	return voltage;
}

float ADCDrv::VPHASE_C()
{
	float voltage = (3.0f * m_Values[ADCVPHASE_C] / 4096.0f) * 9.2546f; // [V]

	return voltage;
}

float ADCDrv::PVDDVoltage()
{
	float voltage = (3.0f * m_Values[ADCPVDD] / 4096.0f) * 9.2546f; // [V]

	return voltage;
}

float ADCDrv::DrvTemperature()
{
	float adcVoltage = (3.0f * m_Values[ADCTEMP] / 4096.0f); // [V]
	float temperature = (2.1f - adcVoltage) * 91.7f; // [10.9mV/C]

	return temperature;
}


// Aux
float ADCDrv::PotVoltage()
{
	float voltage = (3.0f * m_AuxValues[ADCPOT] / 4096.0f); // [V]

	return voltage;
}

unsigned int ADCDrv::PotCount(bool invert)
{
	unsigned int count = m_AuxValues[ADCPOT];
	if( invert ) count = 4095 - count;

	return count;
}

float ADCDrv::CPUTemperature()
{
	float Temperature = 147.5f - ((75.0f * 3.0f * m_AuxValues[ADCCPUTEMP]) / 4096.0f);

	return Temperature;
}
