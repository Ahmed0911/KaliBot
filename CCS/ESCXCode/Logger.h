/*
 * Logger.h
 *
 *  Created on: Nov 12, 2014
 *      Author: User
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#define LOGGERBUFFERSIZE 500
#define LOGHOLDOFFCNT 1000

class Logger
{
public:
	enum ETrigTypes { AUTO, NORMAL, ONCE };

	void Init();
	void AddSample(float ch1, float ch2, float ch3, float ch4);
	void SetTrigger(ETrigTypes trigType, int trigChannel = 1, float level = 1, bool trigRise = true);
	void ClearBuffer();

private:
	// RX Data
	enum ELOGPhases { STOP, ARM, RUN, HOLDOFF };
	ELOGPhases m_LogPhase;

	// buffer
	int m_Index;
	float m_Buffer[4][LOGGERBUFFERSIZE]; // 4 channels

	// trigger
	ETrigTypes m_TrigType;
	int m_TrigChannel;
	float m_TrigLevel;
	bool m_TrigRise;
	float m_TrigLastVal;
	int m_TrigHoldoff;
};

#endif /* LOGGER_H_ */
