/*
 * DBGLed.h
 *
 *  Created on: Jun 26, 2015
 *      Author: Sara
 */

#ifndef DBGLED_H_
#define DBGLED_H_

class DBGLed
{
public:
	enum ELED { LEDDBG = 0, LEDFAULT };

private:
	bool m_LastState[2];

public:
	void Init();
	void Set(ELED led, bool enable);
	void Set(ELED led);
	void Reset(ELED led);
	void Toggle(ELED led);
};

#endif /* DBGLED_H_ */
