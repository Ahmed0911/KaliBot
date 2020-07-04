/*
 * PWMDrv.h
 *
 *  Created on: Jun 4, 2014
 *      Author: User
 */

#ifndef PWMDRV_H_
#define PWMDRV_H_

class PWMDrv
{
public:
	void Init();
	void SetDuty(float dutyA, float dutyB, float dutyC); // BLDC mode
	void Enable(bool enable);

private:
	unsigned long m_PeriodHalf;
};

#endif /* PWMDRV_H_ */
