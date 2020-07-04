/*
 * Pid.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: User
 */

#include "Pid.h"

void Pid::Init(float Kp, float Ki, float Kd, float Kbx, float Ts, float SatMin, float SatMax)
{
	m_Kp = Kp;
	m_Ki = Ki;
	m_Kd = Kd;
	m_Kbx = Kbx;
	m_Kb = m_Kbx * (Ki/Kp); // Ki/Kp = Ti
	m_Ts = Ts;
	m_SatMin = SatMin;
	m_SatMax = SatMax;

	if(m_Kb > 1e5) m_Kb = 0; // safety

	m_Int = 0; // reset integrator
}

void Pid::SetParams(float Kp, float Ki, float SatMin, float SatMax)
{
	m_Kp = Kp;
	m_Ki = Ki;
	m_Kb = m_Kbx * (Ki/Kp); // Ki/Kp = Ti

	if(m_Kb > 1e5) m_Kb = 0; // safety

	m_SatMin = SatMin;
	m_SatMax = SatMax;
}

void Pid::Reset()
{
	m_Int = 0; // reset integrator
}

float Pid::Run(float u)
{
	float Out = 0;

	float sum = m_Kp * u + m_Int; // TODO: add Derivate!

	// saturate output
	if( sum > m_SatMax ) Out = m_SatMax;
	else if( sum < m_SatMin ) Out = m_SatMin;
	else Out = sum;

	// new integrator state
	m_Int += (m_Ki * u  + m_Kb * (Out - sum)) * m_Ts;

	return Out;
}
