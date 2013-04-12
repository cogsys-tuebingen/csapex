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

//common includes and defines
// also C_Mutex, C_Thread, C_Event
// and C_FileSystem

#ifndef _COMMON_H
#define _COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

////////////////////////////////////////////////////////////////////////////////

#ifdef WIN32
#include <windows.h>
#include <winbase.h>
#include <process.h>
typedef __int64          int64_t;
typedef unsigned __int64 uint64_t;

#define mssleep(x) Sleep(x)

#else
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <limits.h>
#include <sys/time.h>

struct POINT
{
  int32_t x;
  int32_t y;
};

#include <strings.h>
#define _stricmp strcasecmp
#define _snprintf snprintf

#define mssleep(x) usleep((x)*1000)

#define MAX_PATH 2048

#endif

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif


////////////////////////////////////////////////////////////////////////////////

//#define PI 3.1415926535

#include "fz_types.h"

union AllPtrType
{
  uint8_t*  u8ptr;
  int8_t*   s8ptr;
  uint16_t* u16ptr;
  int16_t*  s16ptr;
  uint32_t* u32ptr;
  int32_t*  s32ptr;
  uint64_t* u64ptr;
  int64_t*  s64ptr;
  void*     vptr;
};

struct S_Rect
{
  int x, y;
  int width, height;
};

////////////////////////////////////////////////////////////////////////////////
class C_Mutex
{
public:
  C_Mutex();
  ~C_Mutex();
  void Enter();
  void Leave();
  bool TryEnter();
private:
#ifdef WIN32
  HANDLE m_hMutex;
#else
  pthread_mutex_t m_hMutex;
#endif
};
//this class supports multiple threads

////////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
#define THREAD_RET_TYPE DWORD
#define THREAD_RET DWORD WINAPI
#define THREAD_RET_0 0
typedef DWORD (WINAPI* THREADTYPE)(void* pParam);
#else
#define THREAD_RET_TYPE void*
#define THREAD_RET void*
#define THREAD_RET_0 NULL
typedef void* (*THREADTYPE)(void* pParam);
#endif

class C_Thread
{
public:
  C_Thread(THREADTYPE i_pThread, void* i_pParam);
  ~C_Thread();
  bool WaitForThreadExit(int i_iTimeout = -1);
  
  static int NumberOfCPUs();
private:
#ifdef WIN32
  HANDLE m_hThread;
#else
  pthread_t m_hThread;
#endif
};
//currently this class supports one thread waiting

////////////////////////////////////////////////////////////////////////////////
class C_Event
{
public:
  C_Event();
  ~C_Event();
  void Signal();
  bool Wait(int i_iTimeout = -1);
private:
#ifdef WIN32
  HANDLE m_hEvent;
#else
  pthread_mutex_t m_hMutex;
  pthread_cond_t  m_hCond;
  volatile bool m_bSignaled;
#endif
};
//currently this class supports one thread signaling and one thread waiting

class C_FileSystem
{
public:
  //functions to find files in a directory
  static void* FileSearchOpen(char* i_szPath, char* i_szWild);
  static bool FileSearchNext(void* i_pHandle, char* o_szFilename, int i_iStringSize);
  static void FileSearchClose(void* i_pHandle);
  
  static void CreateDir(char* o_szDirname);
  
private:
};

#endif
