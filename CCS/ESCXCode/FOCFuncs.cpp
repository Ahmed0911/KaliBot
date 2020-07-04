/*
 * FOCFuncs.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: User
 */

#include "FOCFuncs.h"
#include <math.h>
// CMSIS Header (See spma041e.pdf)
//#include "arm_math.h"

void FOCFuncs::Svgen(SAlphaBeta ipark, SDutyABC& dutyOut)
{
	int Sector = 0;  // Sector

	float temp_sv1 = (ipark.Beta * 0.5f); 			// divide by 2
	float temp_sv2 = (0.8660254f * ipark.Alpha);		// 0.8660254 = sqrt(3)/2
	float t1, t2 = 0;

	// Inverse clarke transformation
	float Va = ipark.Beta;
	float Vb = -temp_sv1 + temp_sv2;
	float Vc = -temp_sv1 - temp_sv2;

	// 60 degree Sector determination
	if ( Va > 0) Sector = 1;
	if ( Vb > 0) Sector = Sector+2;
	if ( Vc > 0) Sector = Sector+4;

	// X,Y,Z (Va,Vb,Vc) calculations X = Va, Y = Vb, Z = Vc
	Va = ipark.Beta;
	Vb = temp_sv1 + temp_sv2;
	Vc = temp_sv1 - temp_sv2;

	// Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)
	switch(Sector)
	{
	case 0:
		dutyOut.DutyA = 0.5;
		dutyOut.DutyB = 0.5;
		dutyOut.DutyC = 0.5;
	  break;
	case 1:  // Sector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)
		t1 = Vc;
		t2 = Vb;
		dutyOut.DutyB = ((1-t1-t2) * 0.5f);		// tbon = (1-t1-t2)/2
		dutyOut.DutyA = dutyOut.DutyB+t1;			 	// taon = tbon+t1
		dutyOut.DutyC = dutyOut.DutyA+t2;			  	// tcon = taon+t2
	  break;
	case 2:  // Sector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)
		t1 = Vb;
		t2 = -Va;
		dutyOut.DutyA = ((1-t1-t2) * 0.5f);		//	taon = (1-t1-t2)/2
		dutyOut.DutyC = dutyOut.DutyA+t1;				//  tcon = taon+t1
		dutyOut.DutyB = dutyOut.DutyC+t2;				//  tbon = tcon+t2
	  break;
	case 3:  // Sector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)
		t1 = -Vc;
		t2 = Va;
		dutyOut.DutyA= ((1-t1-t2) * 0.5f);		//	taon = (1-t1-t2)/2
		dutyOut.DutyB = dutyOut.DutyA+t1;				//	tbon = taon+t1
		dutyOut.DutyC = dutyOut.DutyB+t2;				//	tcon = tbon+t2
	  break;
	case 4:  // Sector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)
		t1 = -Va;
		t2 = Vc;
		dutyOut.DutyC= ((1-t1-t2) * 0.5f);		//	tcon = (1-t1-t2)/2
		dutyOut.DutyB = dutyOut.DutyC+t1;				//	tbon = tcon+t1
		dutyOut.DutyA = dutyOut.DutyB+t2;				//	taon = tbon+t2
	  break;
	case 5:  // Sector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)
		t1 = Va;
		t2 = -Vb;
		dutyOut.DutyB= ((1-t1-t2) * 0.5f);		//	tbon = (1-t1-t2)/2
		dutyOut.DutyC = dutyOut.DutyB+t1;				//	tcon = tbon+t2
		dutyOut.DutyA = dutyOut.DutyC+t2;				//	taon = tcon+t2
	  break;
	case 6:  // Sector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)
		t1 = -Vb;
		t2 = -Vc;
		dutyOut.DutyC= ((1-t1-t2) * 0.5f);		//	tcon = (1-t1-t2)/2
		dutyOut.DutyA = dutyOut.DutyC+t1;				//	taon = tcon+t1
		dutyOut.DutyB = dutyOut.DutyA+t2;				//	tbon = taon+t2
	  break;
	} // end of switch block


	//  Convert the unsigned (ranged (0,1)) -> signed format (ranged (-1,1))
	dutyOut.DutyA = ((dutyOut.DutyA - 0.5f) * 2.0f);
	dutyOut.DutyB = ((dutyOut.DutyB - 0.5f) * 2.0f);
	dutyOut.DutyC = ((dutyOut.DutyC - 0.5f) * 2.0f);
}


void FOCFuncs::Clarke(float Ia, float Ib, SAlphaBeta& clarkeOut)
{
	clarkeOut.Alpha = Ia;
	clarkeOut.Beta = (Ia + (Ib * 2.0f)) * 0.57735026918963f;
}

void FOCFuncs::IPark(SDQ input, float angleTheta, SAlphaBeta& iparkOut)
{
    float  angle_temp = angleTheta * 6.2831853f; // Theta [0...1] range
    float sinTh = sinf(angle_temp);
    float cosTh = cosf(angle_temp);

	// CMSIS
	//float angle_temp    = (-1.0f)*(angleTheta-0.5f) * 360.0f;	// Conversion for per unit [0,1] to [-0.5,0.5]
	//float sinTh, cosTh;
	//arm_sin_cos_f32 (angle_temp, &sinTh, &cosTh);
	//cosTh = -1 * cosTh;	// Due to CMSIS Cortex R DSP Library_2_0_0.lib cosine table access method*/

	iparkOut.Alpha = (input.Sd * cosTh) - (input.Sq * sinTh);
	iparkOut.Beta = (input.Sd * sinTh) + (input.Sq * cosTh);
}

void FOCFuncs::Park(SAlphaBeta input, float angleTheta, SDQ& parkOut)
{
	float  angle_temp = angleTheta * 6.2831853f; // Theta [0...1] range
	float sinTh = sinf(angle_temp);
	float cosTh = cosf(angle_temp);

	// CMSIS
	//float angle_temp    = (-1.0f)*(angleTheta-0.5f) * 360.0f;	// Conversion for per unit [0,1] to [-0.5,0.5]
	//float sinTh, cosTh;
	//arm_sin_cos_f32 (angle_temp, &sinTh, &cosTh);
	//cosTh = -1 * cosTh;	// Due to CMSIS Cortex R DSP Library_2_0_0.lib cosine table access method

	parkOut.Sd = (input.Alpha * cosTh) + (input.Beta * sinTh);
	parkOut.Sq = -(input.Alpha * sinTh) + (input.Beta * cosTh);
}
