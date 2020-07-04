/*
 * AS5147UEncoder.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: User
 */

#include "AS5147UEncoder.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"


extern uint32_t g_ui32SysClock;

// SPI
// ---------
// 1MHz, 20MHz max (read only)
// Freescale SPI
// SPO = 1,SPH = 0 -> FRF_MOTO_MODE_2
// SSI2
// PINS:
// PD3 - CLK
// PD2 - CS
// PD1 - MOSI
// PD0 - MISO


bool AS5147EncoDrv::Init()
{
	// Enable
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// SPI Pins
	GPIOPinConfigure(GPIO_PD3_SSI2CLK);
	GPIOPinConfigure(GPIO_PD2_SSI2FSS);
	GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
	GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

	// Configure
	SSIConfigSetExpClk(SSI2_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 5000000, 16);
	// Enable
	SSIEnable(SSI2_BASE);

	// Clear fifo
	ClearFIFO();

	return true;
}


void AS5147EncoDrv::Update()
{
	// read status
    Counter = ReadReg(0x3FFC);
    Velocity = ReadReg(0x3FFF);
}

int AS5147EncoDrv::GetCounter()
{
    return Counter;
}

float AS5147EncoDrv::GetVelocity() // CNT/s
{
    float vel = Velocity;
    if( vel > 8192 ) vel = -(16384 - vel);

    // scale [1098.6837 cnt/bit]
    float velCNTs = vel * 1098.6837;

    return velCNTs;
}

/////////////////////
// SPI Functions
/////////////////////
void AS5147EncoDrv::ClearFIFO()
{
	// Clear FIFO
	uint32_t data;
	while(SSIDataGetNonBlocking(SSI2_BASE, &data));
}

uint16_t AS5147EncoDrv::ReadReg(uint16_t address) // return previous value
{
	uint32_t request = address + 0x4000; // 0x4000 - READ FLAG

	SSIDataPut(SSI2_BASE, request);
	while(SSIBusy(SSI2_BASE));

	uint32_t response;
	SSIDataGet(SSI2_BASE, &response); // get data

	return (response & 0x3FFF);
}

/*void AS5147EncoDrv::WriteReg(unsigned char address, unsigned short data)
{
	uint32_t request = (address << 11) + 0x0000 + data; // 0x0000 - WRITE FLAG

	SSIDataPut(SSI2_BASE, request);
	while(SSIBusy(SSI2_BASE));

	volatile uint32_t response;
	SSIDataGet(SSI2_BASE, (uint32_t*)&response); // dummy
}*/
