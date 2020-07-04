/*
 * AS5147UEncoder.h
 *
 *  Created on: Jul 3, 202
 *      Author: User
 */

#ifndef DRIVERS_AS5147UENCODER_H_
#define DRIVERS_AS5147UENCODER_H_

#include <stdint.h>

class AS5147EncoDrv {
public:
    bool Init();
    void Update();
    int GetCounter(); // CNT
    float GetVelocity(); // CNT/s

private:
    uint16_t ReadReg(uint16_t address);
	//void WriteReg(unsigned char address, unsigned short data);
	void ClearFIFO();

	uint16_t Counter;
	uint16_t Velocity;
};

#endif /* DRIVERS_AS5147UENCODER_H_ */
