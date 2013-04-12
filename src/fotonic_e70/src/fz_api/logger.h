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

#ifndef __LOGGER_H
#define __LOGGER_H

#include "common.h" //mutex

#define LOG_NONE  0x0000
#define LOG_ERROR 0x0001
#define LOG_WARN  0x0002
#define LOG_INFO  0x0004
#define LOG_TRACE 0x0008
#define LOG_ALL   0xFFFF

#define LOG_TO_FILE      0x1000
#define LOG_TO_STDOUT    0x2000
#define LOG_OPEN_CONSOLE 0x4000

typedef void (*LOGCALLBACKTYPE)(char* szMetadata, char* szMessage);

class C_Logger
{
public:
  C_Logger();
  ~C_Logger();
  
  void SetLogLevel(int iLogLevel); //default is LOG_ERROR
  void SetOutLevel(int iOutLevel); //default is LOG_TO_STDOUT
  void AssignFile(const char* szFilename);
  void AssignCallback(LOGCALLBACKTYPE pFunction);
  
  void Print(int iLevel, const char* szFmt, ...);
  static void PrintToFile(char* szFilename, const char* szFmt, ...);
private:

  int   m_iLogLevel;
  int   m_iOutLevel;
  char*  m_szFilename;
  bool  m_bCreatedConsole;
  
  LOGCALLBACKTYPE m_pLogCallback;
  
  //thread safety
  C_Mutex* m_pMutex;
};

#endif //__LOGGER_H
