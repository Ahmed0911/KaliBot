/*
 * SignalGen.h
 *
 *  Created on: Nov 13, 2014
 *      Author: User
 */

#ifndef SIGNALGEN_H_
#define SIGNALGEN_H_

class SignalGen
{
public:
	void Init();
	void Config(int period, float min, float max);
	float Get();

private:
	int m_Counter;
	int m_Period;
	float m_Min;
	float m_Max;
	float m_Output;
};

#endif /* SIGNALGEN_H_ */
