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

//
// a logger class that can append text to a file and/or stdout.
// it filters text depending on log level type.
//

#include "logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#ifdef WIN32
#include <share.h>
#endif

#define MAXTEXT 5*512

C_Logger::C_Logger()
{
  m_szFilename = NULL;
  
  m_iLogLevel = LOG_ERROR;
  m_iOutLevel = 0;
  m_bCreatedConsole = false;
  m_pLogCallback = NULL;
  
  m_pMutex = new C_Mutex();
}

C_Logger::~C_Logger()
{
  delete[] m_szFilename;
  delete m_pMutex;
  
#ifdef WIN32
  if(m_bCreatedConsole)
  {
    FreeConsole();
  }
#endif
}

void C_Logger::SetLogLevel(int iLogLevel)
{
  m_iLogLevel = iLogLevel & LOG_ALL;
}

void C_Logger::SetOutLevel(int iOutLevel)
{
  m_iOutLevel = iOutLevel & (LOG_TO_FILE | LOG_TO_STDOUT);
  
#ifdef WIN32
  if((iOutLevel & LOG_OPEN_CONSOLE) && AllocConsole())
  {
    freopen("CONOUT$", "wt", stdout); //ignore result
    SMALL_RECT windowSize = {0, 0, 136 - 1, 64 - 1};
    COORD bufferSize = {136, 5000};
    SetConsoleTitle((LPCTSTR)"FZ-API: Debug Console");
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED);
    SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE), bufferSize);
    SetConsoleWindowInfo(GetStdHandle(STD_OUTPUT_HANDLE), true, &windowSize);
    m_bCreatedConsole = true;
  }
#endif
}

void C_Logger::AssignFile(const char* szFilename)
{
  m_pMutex->Enter();
  delete[] m_szFilename;
  if(szFilename == NULL) m_szFilename = NULL;
  else
  {
    int iLen = (int)strlen(szFilename) + 1;
    m_szFilename = new char[iLen];
    strcpy(m_szFilename, szFilename);
  }
  m_pMutex->Leave();
}

void C_Logger::AssignCallback(LOGCALLBACKTYPE pFunction)
{
  m_pMutex->Enter();
  m_pLogCallback = pFunction;
  m_pMutex->Leave();
}

void C_Logger::Print(int iLevel, const char* szFmt, ...)
{
  if(!m_iLogLevel || !m_iOutLevel || !szFmt) return; //fast pre check
  if((iLevel & m_iLogLevel) == 0) return; //level check
  
  char szText[MAXTEXT];
  va_list stPList;
  va_start(stPList, szFmt);
  vsnprintf(szText, MAXTEXT, szFmt, stPList);
  szText[MAXTEXT - 1] = 0;
  va_end(stPList);
  
  //char for log type
  int iLevelStringIndex = 0; //assume LOG_ERROR
  const char* szLevelString = "EWIT";
  if(iLevel & LOG_WARN) iLevelStringIndex = 1;
  if(iLevel & LOG_INFO) iLevelStringIndex = 2;
  if(iLevel & LOG_TRACE) iLevelStringIndex = 3;
  
  //time string
  char szTime[24];
#ifdef WIN32
  SYSTEMTIME stTime;
  GetLocalTime(&stTime);
  sprintf(szTime, "%02d:%02d:%02d.%03d", stTime.wHour, stTime.wMinute, stTime.wSecond, stTime.wMilliseconds);
#else
  struct timeval tv;
  gettimeofday(&tv, NULL);
  struct tm* now = localtime(&tv.tv_sec);
  sprintf(szTime, "%02d:%02d:%02d.%03d", now->tm_hour, now->tm_min, now->tm_sec, (int)(tv.tv_usec / 1000));
#endif
  
  //log to file if assigned
  if(m_iOutLevel & LOG_TO_FILE)
  {
    m_pMutex->Enter();
    if(m_szFilename)
    {
#ifdef WIN32
      FILE* pFile = _fsopen(m_szFilename, "at", _SH_DENYNO);
#else
      FILE* pFile = fopen(m_szFilename, "at");
#endif
      if(pFile)
      {
        fprintf(pFile, "[%c %s] %s", szLevelString[iLevelStringIndex], szTime, szText);
        fclose(pFile);
      }
    }
    m_pMutex->Leave();
  }
  
  //log to callback if assigned
  m_pMutex->Enter();
  if(m_pLogCallback)
  {
    char szMetadata[64];
    sprintf(szMetadata, "%c %s", szLevelString[iLevelStringIndex], szTime);
    m_pLogCallback(szMetadata, szText);
  }
  m_pMutex->Leave();
  
  //log to stdout
  if(m_iOutLevel & LOG_TO_STDOUT) printf("[%c %s] %s", szLevelString[iLevelStringIndex], szTime, szText);
}

void C_Logger::PrintToFile(char* szFilename, const char* szFmt, ...)
{
  if(!szFmt) return;
  
  char szText[MAXTEXT];
  va_list stPList;
  va_start(stPList, szFmt);
  vsnprintf(szText, MAXTEXT, szFmt, stPList);
  szText[MAXTEXT - 1] = 0;
  va_end(stPList);
  
#ifdef WIN32
  FILE* pFile = _fsopen(szFilename, "at", _SH_DENYNO);
#else
  FILE* pFile = fopen(szFilename, "at");
#endif
  if(pFile)
  {
    fprintf(pFile, "%s", szText);
    fclose(pFile);
  }
}
