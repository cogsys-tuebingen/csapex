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

#include "timer.h"

#ifdef WIN32
C_Timer::C_Timer()
{
  LARGE_INTEGER stTemp;
  
  m_iHiresTimer = QueryPerformanceFrequency(&stTemp);
  if(!m_iHiresTimer)
  {
    m_qFreq = CLK_TCK;
    m_qLastTime = clock();
  }
  else
  {
    m_qFreq = stTemp.QuadPart;
    QueryPerformanceCounter(&stTemp);
    m_qLastTime = stTemp.QuadPart;
  }
  
  m_dPeekTime = 0.0;
}

double C_Timer::GetInterval()
{
  LARGE_INTEGER stTemp;
  INT64 qCurTime, qDiffTime;
  
  if(!m_iHiresTimer)
  {
    qCurTime = clock();
    if(qCurTime >= m_qLastTime) qDiffTime = qCurTime - m_qLastTime; //normal
    else qDiffTime = (0xFFFFFFFF - m_qLastTime) + qCurTime;         //overflow
  }
  else
  {
    QueryPerformanceCounter(&stTemp);
    qCurTime = stTemp.QuadPart;
    if(qCurTime >= m_qLastTime) qDiffTime = qCurTime - m_qLastTime; //normal
    else qDiffTime = (0xFFFFFFFFFFFFFFFF - m_qLastTime) + qCurTime; //overflow
  }
  m_qLastTime = qCurTime;
  
  double dTime = ((double)qDiffTime / ((double)m_qFreq * 0.001));
  dTime += m_dPeekTime;
  m_dPeekTime = 0.0;
  return dTime;
}

double C_Timer::PeekInterval()
{
  m_dPeekTime = GetInterval();
  return m_dPeekTime;
}

void C_Timer::Reset()
{
  LARGE_INTEGER stTemp;
  
  if(!m_iHiresTimer)
  {
    m_qLastTime = clock();
  }
  else
  {
    QueryPerformanceCounter(&stTemp);
    m_qLastTime = stTemp.QuadPart;
  }
  m_dPeekTime = 0.0;
}

#else
C_Timer::C_Timer()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  m_stLastTime = tv;

  m_dPeekTime = 0.0;
}

double C_Timer::GetInterval()
{
  struct timeval tv;
  double dTime;
  gettimeofday(&tv, NULL);
  dTime  = (double)(tv.tv_sec - m_stLastTime.tv_sec) * 1000; //ms because of sec change
  dTime += (double)((tv.tv_usec - m_stLastTime.tv_usec) / 1000); //ms because of usec change
  if(dTime == 0.0) dTime += (1.0 / 1000.0); //bad cheat if too fast computer/too bad resolution in gettimeofday
  m_stLastTime = tv;

  dTime += m_dPeekTime;
  m_dPeekTime = 0.0;
  return dTime;
}

double C_Timer::PeekInterval()
{
  m_dPeekTime = GetInterval();
  return m_dPeekTime;
}

void C_Timer::Reset()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  m_stLastTime = tv;

  m_dPeekTime = 0.0;
}

#endif

C_Timer::~C_Timer()
{
}
