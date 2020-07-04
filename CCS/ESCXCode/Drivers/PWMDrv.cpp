/*
 * PWMDrv.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: User
 */

#include "PWMDrv.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

extern uint32_t g_ui32SysClock;

// 20KHz PWM frequency
#define PWMFREQ 20000

// Maximum 140kHz->with disabled perf. timer

// PWM-AL - PG0 - PWM4
// PWM-AH - PG1 - PWM5
// PWM-BL - PF2 - PWM2
// PWM-BH - PF3 - PWM3
// PWM-CL - PF0 - PWM0
// PWM-CH - PF1 - PWM1
// ENABLE - PF4

void PWMDrv::Init()
{
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1); // same clock as system clock
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // PWM Module 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // PF0 to PF4
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG); // PG0 to PG1

	GPIOPinConfigure(GPIO_PG0_M0PWM4); // PWMAL
	GPIOPinConfigure(GPIO_PG1_M0PWM5); // PWMAH
	GPIOPinConfigure(GPIO_PF2_M0PWM2); // PWMBL
	GPIOPinConfigure(GPIO_PF3_M0PWM3); // PWMBH
	GPIOPinConfigure(GPIO_PF0_M0PWM0); // PWMCL
	GPIOPinConfigure(GPIO_PF1_M0PWM1); // PWMCH

	GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0 | GPIO_PIN_1 );
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);


	unsigned long ulPeriod = g_ui32SysClock / PWMFREQ;
	m_PeriodHalf = ulPeriod/2;

	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ulPeriod); // PWM-A
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ulPeriod); // PWM-B
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ulPeriod); // PWM-C

	// set default PWM duty (50%)
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ulPeriod/2); // A
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ulPeriod/2); // B
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ulPeriod/2); // C

	// Set Deadbands
	PWMDeadBandEnable(PWM0_BASE, PWM_GEN_0, 20, 20);
	PWMDeadBandEnable(PWM0_BASE, PWM_GEN_1, 20, 20);
	PWMDeadBandEnable(PWM0_BASE, PWM_GEN_2, 20, 20);

	// enable PWM outputs to PINs
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);

	// enable PWM modules
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	PWMGenEnable(PWM0_BASE, PWM_GEN_2);

	// sync PWM GEN0 and GEN1
	PWMSyncTimeBase(PWM0_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT | PWM_GEN_2_BIT);

	// Enable PINs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable Pin
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
}

// duty range [-1...1], limited: [-0.8...0.8]!!!
void PWMDrv::SetDuty(float dutyA, float dutyB, float dutyC)
{
	unsigned long widthNeutral = m_PeriodHalf;

	// NOTE: INH and INL are swapped on PCB! pulse width is inverted here to compensate that.
	PWMPulseWidthSet( PWM0_BASE, PWM_OUT_4, widthNeutral - (dutyA * m_PeriodHalf)); // phase A
	PWMPulseWidthSet( PWM0_BASE, PWM_OUT_2, widthNeutral - (dutyB * m_PeriodHalf)); // phase B
	PWMPulseWidthSet( PWM0_BASE, PWM_OUT_0, widthNeutral - (dutyC * m_PeriodHalf)); // phase C
}


void PWMDrv::Enable( bool enable)
{
	if(enable) GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
	else GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
}
