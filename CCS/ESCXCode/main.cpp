/*
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "driverlib/adc.h"
#include "driverlib/eeprom.h"

#include "Drivers/DBGLed.h"
#include "Drivers/Timer.h"
#include "Drivers/EtherDriver.h"
#include "Drivers/DRV8305.h"
#include "Drivers/PWMDrv.h"
#include "Drivers/ADCDrv.h"
#include "Drivers/QEIDrv.h"
#include "Drivers/CANDrv.h"
#include "Drivers/AS5147UEncoder.h"

#include "Pid.h"
#include "AvgFilter.h"
#include "FOCFuncs.h"
#include "Logger.h"
#include "CommData.h"
#include "SignalGen.h"


#define MAX_CURR 30
#define CTARRSIZE 20

// Encoder + CS Params (can be set in application and stored to EEPROM)
int EncoCount = 2000;
int PolePairs = 7;
int EncoThetaOffset = 306; // must be calibrated
bool EncoSwap = true;
int EncoVelFilterOrder = 5;
bool UseAS5147Encoder = false;

// Drivers
DBGLed dbgLed;
Timer timerLoop;
EtherDriver etherDrv;
DRV8305 drv8305;
PWMDrv pwmDrv;
ADCDrv adcDrv;
QEIDrv qeiDrv;
Logger logger;
FOCFuncs foc;
SignalGen sigGen;
CAvgFilter avgVel;
CANDrv canDrv;
AS5147EncoDrv as5147Drv;

// Systick
#define SysTickFrequency 1000
volatile bool SysTickIntHit = false;
uint32_t g_ui32SysClock;

// Functions
bool CalibDrvSense();
void SendPeriodicDataEth();
void ProcessCANData();
void SendPeriodicDataCAN();

// DBG Stuff
bool SafetyDBGDisable = false;

//////////////
// CMDs/REFs
//////////////
int OperationMode = 0; // 0 - Disabled, 1 - Current Tuning, 2 - Torque, 3 - Velocity
float REFCurrentA; // Torque Mode, Current [A]
float REFVelocityCNTs; // Velocity Mode [CNT/s]
int REFPositionCNT; // Position Mode (int64???)
float InternalRefCurrent; // calculated current for Speed/Position Mode, Current [unit]


// Data
float VPhase_A = 0;
float VPhase_B = 0;
float VPhase_C = 0;
float Current_A = 0;
float Current_B = 0;
float Current_C = 0;
float CPUTemperature = 0;
float DrvTemperature = 0;
float VPVDD = 0;
float FocTheta = 0; // FOC theta
int EncoderRawCnt = 0; // raw encoder count
float CurrentId = 0;
float CurrentIq = 0;
bool IndexHit = false;
int OldPosition = 0;
int PositionCNT = 0;
int VelOldPos = 0;
int VelocityCNTs = 0;
float VelocityRPM = 0;
bool CSenseRecalibration = false;

// PIDs
Pid currentIdPid;
Pid currentIqPid;
Pid velocityPid;
Pid positionPid;

// Current Tuning Arrays
float CTCurrentId[CTARRSIZE];
float CTCurrentIq[CTARRSIZE];
float CTCurrentRefId[CTARRSIZE];
float CTCurrentRefIq[CTARRSIZE];
int CTIndex = 0;
float CTCurrentIdCopy[CTARRSIZE];
float CTCurrentIqCopy[CTARRSIZE];
float CTCurrentRefIdCopy[CTARRSIZE];
float CTCurrentRefIqCopy[CTARRSIZE];


// Perf Data
int MainLoopCounter = 0;
float PerfLoopTimeMS;
float PerfCpuTimeMS;
float PerfCpuTimeMSMAX;
float PerfADCTimeus;

// EEPROM stuff
#define EEPROMKEY 0xA5A58832
void ReadMotorParamsFromFlash();

int main(void)
{
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	FPULazyStackingEnable();

	// Ensure that ext. osc is used!
	SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

	// set clock
	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	// Set default params and reload from EEPROM if available
	currentIdPid.Init(0.1, 500, 0, 0.8, 1/20000.0f, -0.8, 0.8);
	currentIqPid.Init(0.1, 500, 0, 0.8, 1/20000.0f, -0.8, 0.8);
	velocityPid.Init(0.002, 0.1, 0, 0.8, 1/1000.0f, -20, 20);
	positionPid.Init(50, 0, 0, 0.8, 1/1000.0f, -25000, 25000); // possible overvoltage on braking!
	ReadMotorParamsFromFlash(); // read from flash

	// Init
	dbgLed.Init();
	timerLoop.Init();
	etherDrv.Init();
	drv8305.Init();
	pwmDrv.Init();
    canDrv.Init();
    qeiDrv.Init(true, EncoSwap, EncoCount);
    logger.Init();
    logger.SetTrigger(Logger::NORMAL, 1, 0.05, true);
    sigGen.Init();
    avgVel.Init(EncoVelFilterOrder);
    as5147Drv.Init();
    adcDrv.Init();
    pwmDrv.SetDuty(0, 0, 0);
    pwmDrv.Enable(true);


	// Systick
	SysTickPeriodSet(g_ui32SysClock/SysTickFrequency);
	SysTickIntEnable();
	SysTickEnable();

	// Force Calibration
	//CalibDrvSense();

	// Master INT Enable
	IntMasterEnable();

	while(1)
	{
		timerLoop.Start(); // start timer
		MainLoopCounter++;

		// Make CT Local Copy
		IntMasterDisable();
		memcpy(CTCurrentIdCopy, CTCurrentId, sizeof(CTCurrentId));
		memcpy(CTCurrentIqCopy, CTCurrentIq, sizeof(CTCurrentIq));
		memcpy(CTCurrentRefIdCopy, CTCurrentRefId, sizeof(CTCurrentRefIdCopy));
		memcpy(CTCurrentRefIqCopy, CTCurrentRefIq, sizeof(CTCurrentRefIqCopy));
		CTIndex = 0;
		IntMasterEnable();

		// Get Data
		VPhase_A = adcDrv.VPHASE_A();
		VPhase_B = adcDrv.VPHASE_B();
		VPhase_C = adcDrv.VPHASE_C();
		VPVDD = adcDrv.PVDDVoltage();
		DrvTemperature = adcDrv.DrvTemperature();
		CPUTemperature = adcDrv.CPUTemperature();

		// CAN
		canDrv.Update();
		ProcessCANData();

		// Position & Velocity
		if( UseAS5147Encoder)
		{
		    int cntEnco = as5147Drv.GetCounter();
		    int deltaPos = cntEnco - OldPosition;
            OldPosition = cntEnco;
            if( deltaPos > (EncoCount/2) ) deltaPos -= EncoCount;       // wrap
            if( deltaPos < -(EncoCount/2) ) deltaPos += EncoCount;
            PositionCNT += deltaPos;
			EncoderRawCnt = cntEnco;
			IndexHit = true;

			// Velocity
			VelocityCNTs = as5147Drv.GetVelocity();
			VelocityRPM = 60.0f * VelocityCNTs / EncoCount;
		}
		else
		{
			int cntEnco = qeiDrv.GetCounter();
			int deltaPos = cntEnco - OldPosition;
			OldPosition = cntEnco;
			if( deltaPos > (EncoCount/2) ) deltaPos -= EncoCount; 		// wrap
			if( deltaPos < -(EncoCount/2) ) deltaPos += EncoCount;
			PositionCNT += deltaPos;
			EncoderRawCnt = cntEnco;
			IndexHit = qeiDrv.IndexDetected();

			// Velocity
			//VelocityCNTs = qeiDrv.GetVelocity();
            int velCnt = PositionCNT - VelOldPos;
            VelOldPos = PositionCNT;
            VelocityCNTs = avgVel.Add(velCnt) * 1000;
            VelocityRPM = 60.0f * VelocityCNTs / EncoCount;
		}

		// Modes
		if( OperationMode == 0 ) // Off
		{
			InternalRefCurrent = 0;
		}
		else if( OperationMode == 1 ) // Current Tuning, handled in Current Lopp
		{
			InternalRefCurrent = 0;
		}
		else if( OperationMode == 2 ) // Torque Mode
		{
			InternalRefCurrent = REFCurrentA/MAX_CURR; // scale to [unit, -1 to 1]
		}
		else if( OperationMode == 3 ) // Velocity Mode
		{
			// speed controller
			float eVel = REFVelocityCNTs - VelocityCNTs;
			float yVel = velocityPid.Run(eVel);
			InternalRefCurrent = yVel/MAX_CURR; // scale to [unit, -1 to 1]
		}
		else if( OperationMode == 4 ) // Position Mode
		{
			// position controller
			int ePos = REFPositionCNT - PositionCNT;
			float yPos = positionPid.Run(ePos);

			// velocity controller
			float eVel = yPos - VelocityCNTs;
			float yVel = velocityPid.Run(eVel);
			InternalRefCurrent = yVel/MAX_CURR; // scale to [unit, -1 to 1]
		}


		// Update
		pwmDrv.Enable(!SafetyDBGDisable);
		drv8305.Update();

		// send periodic ethernet data
		SendPeriodicDataEth();

		// send periodic CAN Data
		SendPeriodicDataCAN();

		// CSense Recalibration
		if( CSenseRecalibration )
		{
			CalibDrvSense();
			CSenseRecalibration = false;
		}

		// DBG LED
		if( (IndexHit && (MainLoopCounter % 50 ) == 0)  )
		{
			dbgLed.Toggle(DBGLed::LEDDBG);
		}
		else if( (MainLoopCounter % 250 ) == 0)
		{
			dbgLed.Toggle(DBGLed::LEDDBG);
		}

		// Get CPU Time
		PerfCpuTimeMS = timerLoop.GetUS()/1000.0f;
		if( PerfCpuTimeMS > PerfCpuTimeMSMAX ) PerfCpuTimeMSMAX = PerfCpuTimeMS;
		// wait next
		while(!SysTickIntHit);
		SysTickIntHit = false;
		// Get total loop time
		PerfLoopTimeMS = timerLoop.GetUS()/1000.0f;
	}
}

// Process CAN Data
#define CANBASEADR 0x100
void ProcessCANData()
{
	int id, len;
	unsigned char msg[8];
	while( canDrv.GetMessage(id, msg, len) == true )
	{
		// process message
		if( id == (CANBASEADR + 0x10))
		{
			// posref cmd
			/*int enabled;
			int positionRefCnt;
			memcpy(&enabled, &msg[0], 4);
			memcpy(&positionRefCnt, &msg[4], 4);
			REFPositionCNT = positionRefCnt;
			if( enabled ) OperationMode = 4;
			else OperationMode = 0;*/

		    // velref cmd
		    int enabled;
		    int velocityRefCnt;
		    memcpy(&enabled, &msg[0], 4);
		    memcpy(&velocityRefCnt, &msg[4], 4);
		    REFVelocityCNTs = velocityRefCnt;
		    if( enabled ) OperationMode = 3;
		    else OperationMode = 0;
		}
	};
}

void SendPeriodicDataCAN(void)
{
	if( (MainLoopCounter%10) == 0 ) // 10 msec period
	{
		unsigned char dataToSend[8];
		memcpy(&dataToSend[0], &OperationMode, 4);
		int indexHit = IndexHit;
		memcpy(&dataToSend[4], &indexHit, 4);
		canDrv.SendMessage(CANBASEADR + 0x0, dataToSend, 8); // status
		memcpy(&dataToSend[0], &PositionCNT, 4);
		canDrv.SendMessage(CANBASEADR + 0x1, dataToSend, 4); // Position
		memcpy(&dataToSend[0], &CurrentIq, 4);
		canDrv.SendMessage(CANBASEADR + 0x2, dataToSend, 4); // Current
	}
}

// Read Flash Data
void ReadMotorParamsFromFlash()
{
	SCommParams params;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	if( EEPROMInit() == EEPROM_INIT_OK )
	{
		unsigned int key;
		EEPROMRead((uint32_t*)&key, 0x0000, sizeof(unsigned int) );
		if( key == EEPROMKEY )
		{
			// eeprom data ok, load config
			EEPROMRead((uint32_t*)&params, sizeof(unsigned int), sizeof(params) );

			// set params
			EncoCount = params.EncoCount;
			PolePairs = params.PolePairs;
			EncoThetaOffset = params.EncoThetaOffset;
			EncoSwap = (params.EncoSwap != 0);
			EncoVelFilterOrder = params.EncoVelFilterOrder;
			UseAS5147Encoder = (params.UsePotAsEncoder !=0 );
			adcDrv.CalibCSA = params.ADCCalibCSA;
			adcDrv.CalibCSB = params.ADCCalibCSB;
			adcDrv.CalibCSC = params.ADCCalibCSC;

			currentIdPid.SetParams(params.Kp_current, params.Ki_current, -0.8, 0.8);
			currentIqPid.SetParams(params.Kp_current, params.Ki_current, -0.8, 0.8);
			velocityPid.SetParams(params.Kp_velocity, params.Ki_velocity, -params.MaxCurrentA_velocity, params.MaxCurrentA_velocity);
			positionPid.SetParams(params.Kp_position, params.Ki_position, -params.MaxVelocityCNTs_position, params.MaxVelocityCNTs_position);
		}
	}
}

// Online Current Sense Calibration (used in Calib, 0x33 command)
bool CalibDrvSense()
{
	drv8305.CalibEnable(true);
	SysCtlDelay(g_ui32SysClock/100/3); // 10ms delay

	unsigned int csA = 0;
	unsigned int csB = 0;
	unsigned int csC = 0;

	for(int i=0; i!=1000; i++)
	{
		SysCtlDelay(g_ui32SysClock/10000/3); // ~10Khz delay
		//adcDrv.IntHandler();
		csA = csA + adcDrv.GetValue(ADCCSM_A);
		csB = csB + adcDrv.GetValue(ADCCSM_B);
		csC = csC + adcDrv.GetValue(ADCCSM_C);
	}

	drv8305.CalibEnable(false);

	csA = csA / 1000;
	csB = csB / 1000;
	csC = csC / 1000;

	adcDrv.CalibCSA = csA;
	adcDrv.CalibCSB = csB;
	adcDrv.CalibCSC = csC;

	return true;
}

void SendPeriodicDataEth(void)
{
	// Fill data
	SCommEthData data;
	data.LoopCounter = MainLoopCounter;

	// Current Data
	data.DrvTemperature = DrvTemperature;
	data.CPUTemperature = CPUTemperature;
	data.VPhase_A = VPhase_A;
	data.VPhase_B = VPhase_B;
	data.VPhase_C = VPhase_C;
	data.VPVDD = VPVDD;
	data.Current_A = Current_A;
	data.Current_B = Current_B;
	data.Current_C = Current_C;
	data.OperationMode = OperationMode;
	data.EncoderLocked = IndexHit;
	data.FocTheta = FocTheta;
	data.EncoderRawCnt = EncoderRawCnt;
	data.CurrentId = CurrentId;
	data.CurrentIq = CurrentIq;
	data.VelocityCNTs = VelocityCNTs;
	data.VelocityRPM = VelocityRPM;
	data.PositionCNT = PositionCNT;
	memcpy(data.CTCurrentId, CTCurrentIdCopy, sizeof(CTCurrentId));
	memcpy(data.CTCurrentIq, CTCurrentIqCopy, sizeof(CTCurrentIq));
	memcpy(data.CTCurrentRefId, CTCurrentRefIdCopy, sizeof(CTCurrentRefId));
	memcpy(data.CTCurrentRefIq, CTCurrentRefIqCopy, sizeof(CTCurrentRefIq));

    // Eth Data + Perf
	data.EthReceivedCount = etherDrv.ReceivedFrames;
	data.EthSentCount = etherDrv.SentFrames;
    data.PerfCpuTimeMS = PerfCpuTimeMS;
	data.PerfCpuTimeMSMAX = PerfCpuTimeMSMAX;
	data.PerfLoopTimeMS = PerfLoopTimeMS;
	data.PerfADCTimeus = PerfADCTimeus;

	// send packet (type 0x20 - data)
	etherDrv.SendPacket(0x20, (char*)&data, sizeof(data));
}

// Process commands received from Ethernet
void ProcessCommand(int cmd, unsigned char* data, int dataSize)
{
	switch( cmd )
	{
		case 0x30:
		{
			SCommCTCmd ctCmd;
			memcpy(&ctCmd, data, dataSize);

			sigGen.Config(100, ctCmd.CTRefMin/MAX_CURR, ctCmd.CTRefMax/MAX_CURR); // set sigGen
			currentIdPid.SetParams(ctCmd.Kp_current, ctCmd.Ki_current, -0.8, 0.8); // set CT params
			currentIqPid.SetParams(ctCmd.Kp_current, ctCmd.Ki_current, -0.8, 0.8);
			EncoThetaOffset = ctCmd.EncoThetaOffset;

			if( ctCmd.Enable ) OperationMode = 1; // enable CT mode
			else OperationMode = 0;

			break;
		}

		case 0x31:
		{
			SCommVTCmd vtCmd;
			memcpy(&vtCmd, data, dataSize);

			velocityPid.SetParams(vtCmd.Kp_velocity, vtCmd.Ki_velocity, -vtCmd.MaxCurrentA_velocity, vtCmd.MaxCurrentA_velocity);

			break;
		}

		case 0x32:
		{
			SCommPTCmd ptCmd;
			memcpy(&ptCmd, data, dataSize);

			positionPid.SetParams(ptCmd.Kp_position, ptCmd.Ki_position, -ptCmd.MaxVelocityCNTs_position, ptCmd.MaxVelocityCNTs_position);

			break;
		}

		case 0x33:
		{
			CSenseRecalibration = true;
			break;
		}

		case 0x40:
		{
			SCommRefCmd refCmd;
			memcpy(&refCmd, data, dataSize);
			REFCurrentA = refCmd.RefCurrentA;
			REFVelocityCNTs = refCmd.RefVelocityCNTs;
			REFPositionCNT = refCmd.RefPositionCNT;

			if( refCmd.Enable ) OperationMode = refCmd.NewOpMode; // enable request mode
			else OperationMode = 0;

			break;
		}

		case 0x50:
		{
			// Read Params
			SCommParams params;

			// Encoder
			params.EncoCount = EncoCount;
			params.PolePairs = PolePairs;
			params.EncoThetaOffset = EncoThetaOffset;
			params.EncoSwap = EncoSwap;
			params.EncoVelFilterOrder = EncoVelFilterOrder;
			params.UsePotAsEncoder = UseAS5147Encoder;
			// Current Sense Calib
			params.ADCCalibCSA = adcDrv.CalibCSA;
			params.ADCCalibCSB = adcDrv.CalibCSB;
			params.ADCCalibCSC = adcDrv.CalibCSC;
			// Current PID
			params.Kp_current = currentIdPid.m_Kp;
			params.Ki_current = currentIdPid.m_Ki;
			// Velocity PID
			params.Kp_velocity = velocityPid.m_Kp;
			params.Ki_velocity = velocityPid.m_Ki;
			params.MaxCurrentA_velocity = velocityPid.m_SatMax;
			// Position PID
			params.Kp_position = positionPid.m_Kp;
			params.Ki_position = positionPid.m_Ki;
			params.MaxVelocityCNTs_position = positionPid.m_SatMax;

			// send packet (type 0x21 - Params Read)
			etherDrv.SendPacket(0x21, (char*)&params, sizeof(params));

			break;
		}

		case 0x51:
		{
			// Store Params To flash
			SCommParams params;
			memcpy(&params, data, dataSize);

			OperationMode = 0; // kill motor when saving and restarting

			// Store to flash (and restart?)
			unsigned int key = EEPROMKEY;

			EEPROMProgram((uint32_t*)&key, 0x0000, sizeof(unsigned int) );
			EEPROMProgram((uint32_t*)&params, sizeof(unsigned int), sizeof(params) );

			// TODO : RESTART???
		}
	}
}

///////////////
// INTERRUPTS
///////////////
extern "C" void SysTickIntHandler(void)
{
	SysTickIntHit = true;
}

//int focCounterX = 0;
//*****************************************************************************
// The interrupt handler for FOC ADC Ctrl, 20kHz
//*****************************************************************************
extern "C" void ADC0IntHandler(void)
{
	// clear ADC interrupt
	ADCIntClear(ADC0_BASE, 0);

	Timer adc;
	adc.Start(); // [0.64us] - with GetuS()

	if( UseAS5147Encoder )
	{
	    as5147Drv.Update();
	}

	// Calculate Theta
	if( IndexHit ) // motor locked
	{
		int encoPosition = 0;
		if( UseAS5147Encoder ) encoPosition = as5147Drv.GetCounter();
		else encoPosition = qeiDrv.GetCounter();

		int EncoElectricCount = EncoCount / PolePairs; // 2000(encototalcount) / 7(numofpolepairs) = 286
		float elecFixed = (float)((encoPosition - EncoThetaOffset) % EncoElectricCount) / EncoElectricCount;
		if(elecFixed > 1 ) elecFixed-=1; // wrap
		else if(elecFixed < 0 ) elecFixed+=1;
		FocTheta = elecFixed;
	}

	float currentRefId = 0;
	float currentRefIq = 0;
	if( OperationMode == 1 ) // CURRENTTUNNING
	{
		currentRefId = sigGen.Get();
		currentRefIq = 0;
		FocTheta = 0; // Zero theta in current tuning!
	}
	else if( OperationMode == 2) // Torque Mode
	{
		currentRefId = 0;
		currentRefIq = InternalRefCurrent;
	}
	else if( OperationMode == 3) // Velocity Mode
	{
		currentRefId = 0;
		currentRefIq = InternalRefCurrent; // calculated in main loop
	}
	else if( OperationMode == 4) // Position Mode
	{
		currentRefId = 0;
		currentRefIq = InternalRefCurrent; // calculated in main loop
	}

	// call ADC INT handler (read new ADC values)
	adcDrv.IntHandler(); // [2.88us]

	// get currents
	Current_A = adcDrv.CSM_A(); // [0.64us]
	Current_B = adcDrv.CSM_B(); // [0.64us]
	Current_C = adcDrv.CSM_C(); // [0.64us]

	// CLARK
	SAlphaBeta clarkeCur;
	foc.Clarke(Current_A/MAX_CURR, Current_B/MAX_CURR, clarkeCur); // [1.10us] rescale to [-1, 1]

	// PARK
	SDQ parkOut;
	foc.Park(clarkeCur, FocTheta, parkOut); // [3.2us]

	CurrentId = parkOut.Sd * MAX_CURR; // convert to [A]
	CurrentIq = parkOut.Sq * MAX_CURR; // convert to [A]

	// PIDs Current Ctrl
	float ecurrId = currentRefId - parkOut.Sd; // [0.04us]
	float ycurrD = currentIdPid.Run(ecurrId); // [1.24us]
	float ecurrIq = currentRefIq - parkOut.Sq; // [0.04us]
	float ycurrQ = currentIqPid.Run(ecurrIq); // [1.24us]

	// IPARK
	SDQ input;
	input.Sd = ycurrD;
	input.Sq = ycurrQ;

	SAlphaBeta ipark;
	foc.IPark(input, FocTheta, ipark); // [3.2us, (without CMSIS: 14.24us)]

	// SVGEN
	SDutyABC dutyOut;
	foc.Svgen(ipark, dutyOut); // [2.20us]

	// PWM Module
	if( OperationMode == 0)
	{
		pwmDrv.SetDuty(0, 0, 0); // Kill output
		// kill controllers
		currentIdPid.Reset();
		currentIqPid.Reset();
		velocityPid.Reset();
		positionPid.Reset();
	}
	else pwmDrv.SetDuty(dutyOut.DutyA, dutyOut.DutyB, dutyOut.DutyC); // [3.36us]


	//if( (focCounterX++ % 1000) == 0 )
	//{
		//logger.AddSample(dutyOut.DutyA, dutyOut.DutyB, dutyOut.DutyC, dutyOut.DutyA-dutyOut.DutyB);
		//logger.AddSample(FocTheta, dutyOut.DutyA, Ia, Ib);
		//logger.AddSample(FocTheta, dutyOut.DutyA, clarkeCur.Alpha, clarkeCur.Beta);
		//logger.AddSample(FocTheta, dutyOut.DutyA, parkOut.Sd, parkOut.Sq);

		//logger.AddSample(ipark.Alpha, clarkeCur.Alpha, ipark.Beta, clarkeCur.Beta); // [1.28us]
		//logger.AddSample(currentRefId, parkOut.Sd, ipark.Beta, clarkeCur.Beta); // [1.28us] - current ctrl // CURRENTTUNNING: uncomment
		//logger.AddSample(focTheta, dutyOut.DutyA, clarkeCur.Alpha, ipark.Alpha); // [1.28us]
		//logger.AddSample(ycurrD, parkOut.Sd, parkOut.Sq, 0); // [1.28us]
	//}

	// Current Tunning/Logging
	if( CTIndex < CTARRSIZE)
	{
		CTCurrentId[CTIndex] = CurrentId; // [A]
		CTCurrentIq[CTIndex] = CurrentIq; // [A]
		CTCurrentRefId[CTIndex] = currentRefId * MAX_CURR; // [A]
		CTCurrentRefIq[CTIndex] = currentRefIq * MAX_CURR; // [A]
		CTIndex++;
	}

	PerfADCTimeus = adc.GetUS();
}
