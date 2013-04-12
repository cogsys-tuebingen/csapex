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

//classes for OS specific execution controlling functions.
// - thread
// - mutex
// - event

#include "common.h"

////////////////////////////////////////////////////////////////////////////////
C_Mutex::C_Mutex()
{
#ifdef WIN32
  m_hMutex = CreateMutex(NULL, FALSE, NULL);
#else
  pthread_mutex_init(&m_hMutex, NULL);
#endif
}

C_Mutex::~C_Mutex()
{
#ifdef WIN32
  CloseHandle(m_hMutex);
#else
  pthread_mutex_destroy(&m_hMutex);
#endif
}

void C_Mutex::Enter()
{
  //lock
#ifdef WIN32
  WaitForSingleObject(m_hMutex, INFINITE);
#else
  pthread_mutex_lock(&m_hMutex);
#endif
}

bool C_Mutex::TryEnter()
{
  //try lock, returnes true if the mutex can be locked immideatly
#ifdef WIN32
  return (WaitForSingleObject(m_hMutex, 0) == WAIT_OBJECT_0);
#else
  return (pthread_mutex_trylock(&m_hMutex) == 0);
#endif
}

void C_Mutex::Leave()
{
  //unlock
#ifdef WIN32
  ReleaseMutex(m_hMutex);
#else
  pthread_mutex_unlock(&m_hMutex);
#endif
}

////////////////////////////////////////////////////////////////////////////////
C_Thread::C_Thread(THREADTYPE i_pThread, void* i_pParam)
{
#ifdef WIN32
  m_hThread = CreateThread(NULL, 0, i_pThread, i_pParam, 0, NULL);
  SetThreadPriority(m_hThread, THREAD_PRIORITY_NORMAL);
#else
  pthread_attr_t stAttr;
  pthread_attr_init(&stAttr);
  pthread_create(&m_hThread, &stAttr, i_pThread, i_pParam);
  pthread_attr_destroy(&stAttr);
#endif
}

bool C_Thread::WaitForThreadExit(int i_iTimeout)
{
#ifdef WIN32
  DWORD dwError = WaitForSingleObject(m_hThread, i_iTimeout == -1 ? INFINITE : i_iTimeout);
  return (dwError == WAIT_OBJECT_0);
#else
  //no timeout can be used?
  return (pthread_join(m_hThread, NULL) == 0);
#endif
}

C_Thread::~C_Thread()
{
#ifdef WIN32
  CloseHandle(m_hThread);
#else
  //?
#endif
}

int C_Thread::NumberOfCPUs()
{
#ifdef WIN32
  SYSTEM_INFO stSysinfo;
  GetSystemInfo(&stSysinfo);
  return (int)stSysinfo.dwNumberOfProcessors;
#else
  return (int)sysconf(_SC_NPROCESSORS_ONLN);
#endif
}

////////////////////////////////////////////////////////////////////////////////
C_Event::C_Event()
{
#ifdef WIN32
  m_hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
#else
  pthread_mutex_init(&m_hMutex, NULL);
  pthread_cond_init(&m_hCond, NULL);
  m_bSignaled = false;
#endif
}

C_Event::~C_Event()
{
#ifdef WIN32
  CloseHandle(m_hEvent);
#else
  pthread_mutex_lock(&m_hMutex);
  pthread_cond_broadcast(&m_hCond);
  pthread_mutex_unlock(&m_hMutex);
  pthread_cond_destroy(&m_hCond);
  pthread_mutex_destroy(&m_hMutex);
#endif
}

void C_Event::Signal()
{
#ifdef WIN32
  SetEvent(m_hEvent);
#else
  pthread_mutex_lock(&m_hMutex);
  m_bSignaled = true;
  pthread_cond_signal(&m_hCond);
  pthread_mutex_unlock(&m_hMutex);
#endif
}

bool C_Event::Wait(int i_iTimeout)
{
#ifdef WIN32
  DWORD dwError = WaitForSingleObject(m_hEvent, i_iTimeout == -1 ? INFINITE : i_iTimeout);
  return (dwError == WAIT_OBJECT_0);
#else
  int ret = 0;
  timespec stTimeSpec;
  if(i_iTimeout != -1)
  {
    timeval  stTimeVal;
    gettimeofday(&stTimeVal, NULL);
    stTimeSpec.tv_sec = stTimeVal.tv_sec + (i_iTimeout / 1000);
    stTimeSpec.tv_nsec = stTimeVal.tv_usec * 1000 + ((i_iTimeout % 1000) * 1000000);
    if(stTimeSpec.tv_nsec >= 1000000000)
    {
      stTimeSpec.tv_sec += 1;
      stTimeSpec.tv_nsec -= 1000000000;
    }
  }
  pthread_mutex_lock(&m_hMutex);
  //m_bSignaled = false; //reset
  while(!m_bSignaled && ret == 0)
  {
    if(i_iTimeout != -1) ret = pthread_cond_timedwait(&m_hCond, &m_hMutex, &stTimeSpec);
    else ret = pthread_cond_wait(&m_hCond, &m_hMutex);
  }
  m_bSignaled = false; //reset
  pthread_mutex_unlock(&m_hMutex);
  return (ret == 0);
#endif
}

////////////////////////////////////////////////////////////////////////////////
//fileseasch and directory creation
#ifdef WIN32
//windows
struct S_FileSearch
{
  WIN32_FIND_DATA stFindData;
  HANDLE          stFile;
  int             iState; //0 not started, 1st name given
};

void* C_FileSystem::FileSearchOpen(char* i_szPath, char* i_szWild)
{
  if(!i_szPath || !i_szWild) return NULL;
  char szPathAndWild[MAX_PATH];
  sprintf(szPathAndWild, "%s/%s", i_szPath, i_szWild);
  S_FileSearch* pSearch = new S_FileSearch();
  pSearch->iState = 0;
  pSearch->stFile = FindFirstFile(szPathAndWild, &pSearch->stFindData);
  return pSearch;
}

bool C_FileSystem::FileSearchNext(void* i_pHandle, char* o_szFilename, int i_iStringSize)
{
  S_FileSearch* pSearch = (S_FileSearch*)i_pHandle;
  if(!pSearch || !o_szFilename || pSearch->stFile == INVALID_HANDLE_VALUE) return false;
  if(pSearch->iState == 0)
  {
    pSearch->iState++;
    strncpy(o_szFilename, pSearch->stFindData.cFileName, i_iStringSize);
    o_szFilename[i_iStringSize - 1] = 0;
    return true;
  }
  bool bResult = FindNextFile(pSearch->stFile, &pSearch->stFindData) ? true : false;
  if(bResult)
  {
    strncpy(o_szFilename, pSearch->stFindData.cFileName, i_iStringSize);
    o_szFilename[i_iStringSize - 1] = 0;
  }
  return bResult;
}

void C_FileSystem::FileSearchClose(void* i_pHandle)
{
  S_FileSearch* pSearch = (S_FileSearch*)i_pHandle;
  if(!pSearch) return;
  if(pSearch->stFile != INVALID_HANDLE_VALUE) FindClose(pSearch->stFile);
  delete pSearch;
}

void C_FileSystem::CreateDir(char* i_szDirname)
{
  CreateDirectory(i_szDirname, NULL);
}

#else
//linux
#include <dirent.h>
#include <fnmatch.h>
#include <sys/stat.h> //for mkdir

struct S_FileSearch
{
  DIR* pDir;
  char szWild[32];
};

void* C_FileSystem::FileSearchOpen(char* i_szPath, char* i_szWild)
{
  if(!i_szPath || !i_szWild) return NULL;
  S_FileSearch* pSearch = new S_FileSearch();
  pSearch->pDir = opendir(i_szPath);
  strcpy(pSearch->szWild, i_szWild);
  return pSearch;
}

bool C_FileSystem::FileSearchNext(void* i_pHandle, char* o_szFilename, int i_iSize)
{
  bool bFound = false;
  S_FileSearch* pSearch = (S_FileSearch*)i_pHandle;
  if(!pSearch || !o_szFilename || !pSearch->pDir) return false;
  struct dirent* pDE = NULL;
  while(!bFound)
  {
    pDE = readdir(pSearch->pDir);
    if(!pDE) return false;
    if(fnmatch(pSearch->szWild, pDE->d_name, FNM_PATHNAME | FNM_PERIOD | FNM_NOESCAPE) == 0)
    {
      strncpy(o_szFilename, pDE->d_name, i_iSize);
      o_szFilename[i_iSize - 1] = 0;
      bFound = true;
    }
  }
  return true;
}

void C_FileSystem::FileSearchClose(void* i_pHandle)
{
  S_FileSearch* pSearch = (S_FileSearch*)i_pHandle;
  if(!pSearch) return;
  if(pSearch->pDir) closedir(pSearch->pDir);
  delete pSearch;
}

void C_FileSystem::CreateDir(char* i_szDirname)
{
  mkdir(i_szDirname, 0755);
}
#endif
