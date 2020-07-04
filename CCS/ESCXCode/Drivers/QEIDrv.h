/*
 * QEIDrv.h
 *
 *  Created on: Jul 1, 2014
 *      Author: User
 */

#ifndef QEIDRV_H_
#define QEIDRV_H_

class QEIDrv
{
public:
	void Init(bool resetOnIndex, bool invert = false, int encoderCntPerRevolution = 10000);
	int GetCounter();
	float GetVelocity(); // CNT/s
	void SetCounter(int value);
	bool IndexDetected();
};

#endif /* QEIDRV_H_ */
