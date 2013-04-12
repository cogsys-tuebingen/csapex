/*
 * Copyright 2012 Fotonic
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _TIMER_H
#define _TIMER_H

#include "common.h"

class C_Timer
{
public:
  C_Timer();
  ~C_Timer();
  double GetInterval();
  //returns time in milliseconds since last call to GetInterval()
  double PeekInterval();
  //returns time in milliseconds since last call to GetInterval()
  
  void Reset();
  //sets time counter to 0
private:
#ifdef WIN32
  int m_iHiresTimer;
  int64_t m_qFreq, m_qLastTime;
#else
  timeval m_stLastTime;
#endif
  
  double m_dPeekTime;  //milliseconds
};

#endif
