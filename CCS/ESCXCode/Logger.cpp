/*
 * Logger.cpp
 *
 *  Created on: Nov 12, 2014
 *      Author: User
 */

#include "Logger.h"
#include "string.h"

void Logger::Init()
{
	m_LogPhase = Logger::STOP;
	ClearBuffer();
}

void Logger::SetTrigger(ETrigTypes trigType, int trigChannel, float level, bool trigRise)
{
	m_TrigType = trigType;
	m_TrigChannel = trigChannel;
	m_TrigLevel = level;
	m_TrigRise = trigRise;
	if( m_TrigRise ) m_TrigLastVal = 1e10; // avoid triggering on first sample!
	else m_TrigLastVal = -1e10;
	m_LogPhase = Logger::ARM; // arm logger

	ClearBuffer();
}

void Logger::ClearBuffer()
{
	m_Index = 0;
	memset(m_Buffer, 0, sizeof(m_Buffer));
}

void Logger::AddSample(float ch1, float ch2, float ch3, float ch4)
{
	// check trigger
	if( m_LogPhase == Logger::ARM)
	{
		if( m_TrigType == Logger::AUTO ) m_LogPhase = Logger::RUN; // always trigger
		else if( m_TrigType == Logger::NORMAL || m_TrigType == Logger::ONCE)
		{
			float trigCh = ch1;
			if( m_TrigChannel == 2) trigCh = ch2;
			else if( m_TrigChannel == 3) trigCh = ch3;
			else if( m_TrigChannel == 4) trigCh = ch4;

			if( m_TrigRise)
			{
				if( m_TrigLastVal < m_TrigLevel && trigCh > m_TrigLevel ) m_LogPhase = Logger::RUN;
			}
			else
			{
				if( m_TrigLastVal > m_TrigLevel && trigCh < m_TrigLevel ) m_LogPhase = Logger::RUN;
			}
			m_TrigLastVal = trigCh;
		}
	}

	// sample in RUN mode
	if( m_LogPhase == Logger::RUN )
	{
		m_Buffer[0][m_Index] = ch1;
		m_Buffer[1][m_Index] = ch2;
		m_Buffer[2][m_Index] = ch3;
		m_Buffer[3][m_Index] = ch4;

		m_Index++;
		if( m_Index >= LOGGERBUFFERSIZE) // buffer full, go into holdoff
		{
			m_LogPhase = Logger::HOLDOFF; // acq done
			m_TrigHoldoff = LOGHOLDOFFCNT; // reset holdoff counter [samples]
		}
	}

	if( m_LogPhase == Logger::HOLDOFF)
	{
		if( m_TrigHoldoff > 0 ) m_TrigHoldoff--;
		else
		{
			m_Index = 0;
			if( m_TrigType == Logger::ONCE ) m_LogPhase = Logger::STOP; // done!
			else m_LogPhase = Logger::ARM; // rearm
		}
	}
}
