/*
 * Pid.h
 *
 *  Created on: Nov 17, 2014
 *      Author: User
 */

#ifndef PID_H_
#define PID_H_

class Pid
{
public:
	void Init(float Kp, float Ki, float Kd, float Kbx, float Ts, float SatMin, float SatMax);
	void SetParams(float Kp, float Ki, float SatMin, float SatMax);
	float Run(float u);
	void Reset();

public:
	float m_Kp; // P gain
	float m_Ki; // I gain
	float m_Kd; // D gain
	float m_SatMin; // saturation min
	float m_SatMax; // saturation max

private:
	float m_Kb; // antiwindup back-calc (modified with Ki!!!)
	float m_Kbx; // antiwindup back-calc copy (used in SetParams())
	float m_Ts; // sample time [s]
	float m_Int;
};

#endif /* PID_H_ */
