/*
 * ADCDrv.h
 *
 *  Created on: Nov 7, 2014
 *      Author: User
 */

#ifndef ADCDRV_H_
#define ADCDRV_H_

// Main Values
#define ADCCSM_A 0
#define ADCCSM_B 1
#define ADCCSM_C 2
#define ADCVPHASE_A 3
#define ADCVPHASE_B 4
#define ADCVPHASE_C 5
#define ADCPVDD 6
#define ADCTEMP 7

// Aux Values
#define ADCPOT 0
#define ADCCPUTEMP 1


class ADCDrv
{
private:
	unsigned int m_Values[8];
	unsigned int m_AuxValues[4];

public:
	ADCDrv();
	void Init();
	unsigned int GetValue(int channel);
	void IntHandler();

	// Board specific!
	float CSM_A();
	float CSM_B();
	float CSM_C();
	float VPHASE_A();
	float VPHASE_B();
	float VPHASE_C();
	float PVDDVoltage();
	float DrvTemperature();

	float PotVoltage();
	unsigned int PotCount(bool invert);
	float CPUTemperature();

	// calib data
	unsigned int CalibCSA;
	unsigned int CalibCSB;
	unsigned int CalibCSC;
};

#endif /* ADCDRV_H_ */
