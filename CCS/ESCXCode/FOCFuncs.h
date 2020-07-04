/*
 * FOCFuncs.h
 *
 *  Created on: Jan 20, 2015
 *      Author: User
 */

#ifndef FOCFUNCS_H_
#define FOCFUNCS_H_

struct SDQ
{
	float Sd;
	float Sq;
};

struct SAlphaBeta
{
	float Alpha;
	float Beta;
};

struct SDutyABC
{
	float DutyA;
	float DutyB;
	float DutyC;
};

class FOCFuncs
{
public:
	void Svgen(SAlphaBeta ipark, SDutyABC& dutyOut);
	void Clarke(float Ia, float Ib, SAlphaBeta& clarkeOut);
	void IPark(SDQ input, float angleTheta, SAlphaBeta& iparkOut);
	void Park(SAlphaBeta input, float angleTheta, SDQ& parkOut);
};

#endif /* FOCFUNCS_H_ */
