/*
 * DRV8305.h
 *
 *  Created on: Apr 14, 2016
 *      Author: User
 */

#ifndef DRIVERS_DRV8305_H_
#define DRIVERS_DRV8305_H_

class DRV8305 {
public:
	bool Init();
	void Update();
	void CalibEnable(bool enable);

private:
	unsigned short ReadReg(unsigned char address);
	void WriteReg(unsigned char address, unsigned short data);
	void ClearFIFO();

public:
	unsigned short StatusWarning;
	unsigned short OVDSFaults;
	unsigned short ICFaults;
	unsigned short VGSFaults;
};

#endif /* DRIVERS_DRV8305_H_ */
