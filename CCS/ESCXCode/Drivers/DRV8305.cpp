/*
 * DRV8305.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: User
 */

#include "DRV8305.h"
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
// SPO = 1,SPH = 1 -> FRF_MOTO_MODE_3
// SSI0
// PINS:
// PA2 - CLK
// PA3 - CS
// PA4 - MOSI
// PA5 - MISO

// PA0 - PWRGD
// PA6 - NFAULT

bool DRV8305::Init()
{
	// Enable
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// SPI Pins
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
	GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

	// Configure
	SSIConfigSetExpClk(SSI0_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 1000000, 16);
	// Enable
	SSIEnable(SSI0_BASE);

	// Clear fifo
	ClearFIFO();

	// Set Registers
	//unsigned short NewGateDrive = 0x0378; // 0.25A sink/source
	unsigned short NewGateDrive = 0x03AB; // 1.0A sink/source
	WriteReg(0x05, NewGateDrive); // HSGateDrive
	WriteReg(0x06, NewGateDrive); // LSGateDrive

	// Activate 3.3V sense clamp (ADC protection)
	unsigned short ICOp = ReadReg(0x09);
	ICOp |= 0x0080;
	WriteReg(0x09, ICOp);

	// Set to 20V/V (30A max -> +/-1.2V span)
	unsigned short ShuntAmpCtrl = 0x0015;
	WriteReg(0x0A, ShuntAmpCtrl);

	// Set VDS Over current protection
	unsigned short VDSSense = 0x0000; // 0x06V protection (~120A on 0.5mohm Rds)
	WriteReg(0x0C, VDSSense);

	return true;
}

void DRV8305::CalibEnable(bool enable)
{
	// read reg
	unsigned short ShuntAmpCtrl = ReadReg(0x0A);

	if( enable ) ShuntAmpCtrl |= 0x0700;
	else ShuntAmpCtrl &= ~(0x0700);

	// write reg back
	WriteReg(0x0A, ShuntAmpCtrl);
}

void DRV8305::Update()
{
	// read status
	StatusWarning =  ReadReg(0x01);
	OVDSFaults =  ReadReg(0x02);
	ICFaults = ReadReg(0x03);
	VGSFaults = ReadReg(0x04);
}

void DRV8305::ClearFIFO()
{
	// Clear FIFO
	uint32_t data;
	while(SSIDataGetNonBlocking(SSI0_BASE, &data));
}

unsigned short DRV8305::ReadReg(unsigned char address)
{
	uint32_t request = (address << 11) + 0x8000; // 0x8000 - READ FLAG

	SSIDataPut(SSI0_BASE, request);
	while(SSIBusy(SSI0_BASE));

	uint32_t response;
	SSIDataGet(SSI0_BASE, &response); // get data

	return (response & 0x007FF);
}

void DRV8305::WriteReg(unsigned char address, unsigned short data)
{
	uint32_t request = (address << 11) + 0x0000 + data; // 0x0000 - WRITE FLAG

	SSIDataPut(SSI0_BASE, request);
	while(SSIBusy(SSI0_BASE));

	volatile uint32_t response;
	SSIDataGet(SSI0_BASE, (uint32_t*)&response); // dummy
}
