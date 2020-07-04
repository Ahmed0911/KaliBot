/*
 * DBGLed.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: Sara
 */

#include "DBGLed.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

void DBGLed::Init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);

	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

	Reset(LEDDBG);
	Reset(LEDFAULT);
}

void DBGLed::Set(ELED led, bool enable)
{
	if(enable) Set(led);
	else Reset(led);
}

void DBGLed::Set(ELED led)
{
	switch (led)
	{
		case LEDDBG:
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5 );
			m_LastState[LEDDBG] = true;
			break;

		case LEDFAULT:
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4 );
			m_LastState[LEDFAULT] = true;
			break;
	}
}

void DBGLed::Reset(ELED led)
{
	switch (led)
	{
		case LEDDBG:
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_0 );
			m_LastState[LEDDBG] = false;
			break;

		case LEDFAULT:
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_0 );
			m_LastState[LEDFAULT] = false;
			break;
	}
}

void DBGLed::Toggle(ELED led)
{
	if( m_LastState[led] ) Reset(led);
	else Set(led);
}
