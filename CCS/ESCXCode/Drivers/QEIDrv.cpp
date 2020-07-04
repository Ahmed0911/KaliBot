/*
 * QEIDrv.cpp
 *
 *  Created on: Jul 1, 2014
 *      Author: User
 */

#include "QEIDrv.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/qei.h"
#include "driverlib/sysctl.h"

#define QEIVELFREQHZ 100

void QEIDrv::Init(bool resetOnIndex, bool invert, int encoderCntPerRevolution)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

	// QEI0
	GPIOPinConfigure(GPIO_PL1_PHA0);
	GPIOPinConfigure(GPIO_PL2_PHB0);
	GPIOPinConfigure(GPIO_PL3_IDX0);

	GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_1);
	GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_2);
	GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_3);


    // setup QEI
	uint32_t swap = 0;
	if( invert ) swap = QEI_CONFIG_SWAP;

	if( resetOnIndex )
	{
		QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_RESET_IDX | QEI_CONFIG_QUADRATURE | swap), encoderCntPerRevolution-1);
	}
	else
	{
		QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | swap), 0xffffffff);
	}


	QEIPositionSet(QEI0_BASE, 0); //reset counter to zero
	QEIEnable(QEI0_BASE); // enable

	// Velocity
	QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet()/QEIVELFREQHZ );
	QEIVelocityEnable(QEI0_BASE);
}

bool QEIDrv::IndexDetected()
{
	bool indexHit = ( QEIIntStatus(QEI0_BASE, false) & QEI_INTINDEX );

	return indexHit;
}

int QEIDrv::GetCounter()
{
	int ret = QEIPositionGet(QEI0_BASE);

	return ret;
}

void QEIDrv::SetCounter(int value)
{
	QEIPositionSet( QEI0_BASE, value);
}

float QEIDrv::GetVelocity()
{
	float cnt = (float)QEIVelocityGet(QEI0_BASE) * QEIDirectionGet(QEI0_BASE);

	cnt = cnt * QEIVELFREQHZ; // CNT/s

	return cnt;
}
