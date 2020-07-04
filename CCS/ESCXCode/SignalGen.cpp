/*
 * SignalGen.cpp
 *
 *  Created on: Nov 13, 2014
 *      Author: User
 */

#include "SignalGen.h"

void SignalGen::Init()
{
	m_Counter = 0;
	m_Output = 0;
	m_Min = 0;
	m_Max = 0;
	m_Period = 0;
}

void SignalGen::Config(int period, float min, float max)
{
	m_Counter = 0;
	m_Period = period;
	m_Min = min;
	m_Max = max;
}

float SignalGen::Get()
{
	if( m_Counter == m_Period) m_Output = m_Max;
	if( m_Counter >= (2*m_Period) )
	{
		m_Output = m_Min;
		m_Counter = 0;
	}
	m_Counter++;

	return m_Output;
}
