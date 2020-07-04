/*
 * CommData.h
 *
 *  Created on: May 23, 2016
 *      Author: Ivan
 */

#ifndef COMMDATA_H_
#define COMMDATA_H_

struct SCommEthData
{
	unsigned int LoopCounter;

	// Current Data
	float DrvTemperature;// [°C]
	float CPUTemperature;// [°C]
	float VPhase_A;
	float VPhase_B;
	float VPhase_C;
	float VPVDD;
	float Current_A;
    float Current_B;
    float Current_C;
	int OperationMode;
	int EncoderLocked;
	int EncoderRawCnt;
	float FocTheta;
	float CurrentId;
	float CurrentIq;
	int VelocityCNTs;
	float VelocityRPM;
	int PositionCNT;

	// Current Tunning Arrays
	float CTCurrentId[20];
	float CTCurrentIq[20];
	float CTCurrentRefId[20];
	float CTCurrentRefIq[20];

	// Comm Eth
	unsigned int EthSentCount;
	unsigned int EthReceivedCount;

	// Perf Stuff
	float PerfLoopTimeMS;
	float PerfCpuTimeMS;
	float PerfCpuTimeMSMAX;
	float PerfADCTimeus;
};

// Current Tuning And Encoder Calibration
struct SCommCTCmd
{
	unsigned int Enable; // 0 - disable, 1 - enable
	float CTRefMin; // [A]
	float CTRefMax; // [A]

	float Kp_current;
	float Ki_current;
	int EncoThetaOffset;
};

// Velocity Tuning
struct SCommVTCmd
{
	float Kp_velocity;
	float Ki_velocity;
	float MaxCurrentA_velocity;
};

// Position Tuning
struct SCommPTCmd
{
	float Kp_position;
	float Ki_position;
	float MaxVelocityCNTs_position;
};

// Ref Command with enable/disable
struct SCommRefCmd
{
	 unsigned int Enable; // 0 - disable, 1 - enable
	 int NewOpMode; // same as ethData "OperationMode"
	 float RefCurrentA; // [A]
	 float RefVelocityCNTs; // [CNT/s]
	 int RefPositionCNT; // [CNT]
};

// Params Read/Write
struct SCommParams
{
	// Encoder
	int EncoCount;
	int PolePairs;
	int EncoThetaOffset;
	int EncoSwap;
	int EncoVelFilterOrder;
	int UsePotAsEncoder;

	// Current Sense Calib
	unsigned int ADCCalibCSA;
	unsigned int ADCCalibCSB;
	unsigned int ADCCalibCSC;

	// Current PID
	float Kp_current;
	float Ki_current;

	// Velocity PID
	float Kp_velocity;
	float Ki_velocity;
	float MaxCurrentA_velocity;

	// Position PID
	float Kp_position;
	float Ki_position;
	float MaxVelocityCNTs_position;
};

// Ethernet packets
// data[0] = 0x42; // magic codes
// data[1] = 0x24; // magic codes
// data[2] = TYPE; // [0x10 - PING, 0x20 - DATA...]
// data[3] = data....
#endif /* COMMDATA_H_ */
