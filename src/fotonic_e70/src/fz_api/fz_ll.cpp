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

#include <stdio.h>
#include <vector>

#include "fz_ll_internal.h"
#include "fz_ll.h"

#include "logger.h"

//driver globals
static bool          gbLLDrvCtxInitialized = false;
DRIVER_CONTEXT       gLLDrvCtx;

extern C_Logger*      gpLog; //from fzapi.cpp
extern volatile bool gbIsInitialized; //from fzapi.cpp

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//ethernet setup impl

#include "timer.h"

/*****************************************************************************/
//special ignore feature
bool IsIgnoreNetwork(char aszIgnoreNetworks[16][128], int iNumIgnoreNetworks, char* szNetworkName)
{
  int i;
  for(i = 0; i < iNumIgnoreNetworks; i++)
  {
    if(strcmp(aszIgnoreNetworks[i], szNetworkName) == 0) return true;
  }
  return false;
}
/*****************************************************************************/

bool SendBroadcastToAllInterfaces(FZ_LL_EthPCtoFZBroadCast* pstData, int iLen, int iPort)
{
  char szText[256]; //debug text
  C_Socket* pBcastSend = new C_Socket();
  
  /*****************************************************************************/
  //special ignore feature: read "fz_enum_eth_ignore.txt" to ignore certain interfaces
  int iNumIgnoreNetworks = 0;
  char aszIgnoreNetworks[16][128];
  {
    char szIgnoreFile[MAX_PATH];
    //find path of the exe file using this dll
#ifdef WIN32
    size_t iStringLen = GetModuleFileName(NULL, szIgnoreFile, sizeof(szIgnoreFile));
#else
    size_t iStringLen = 2;
    strcpy(szIgnoreFile, "./");
#endif
    if(iStringLen > 0 && iStringLen < sizeof(szIgnoreFile))
    {
      //we got the name
      szIgnoreFile[iStringLen] = 0;
      //locate the last "\" or "/" in the filename
      char* pLastBackslash = strrchr(szIgnoreFile, '\\');
      if(!pLastBackslash) pLastBackslash = strrchr(szIgnoreFile, '/');
      if(pLastBackslash)
      {
        *pLastBackslash = 0; // cut on last "\" or "/"
      }
      //add file name of ignore file
      strcat(szIgnoreFile, "/fz_enum_eth_ignore.txt");
      //gpLog->Print(LOG_INFO, "[****] FZ_LL SendBroadcastToAllInterfaces: Trying to open file \"%s\"\n", szIgnoreFile);
      //^log just to get to know where the file is attempted to be found if the exe directory is hard to know (matlab)
      
      //read file contents to simple array of strings
      FILE* pFile = fopen(szIgnoreFile, "rt");
      if(pFile)
      {
        gpLog->Print(LOG_INFO, "[____] FZ_LL SendBroadcastToAllInterfaces: Using found file \"%s\" to ignore broadcasts on given interfaces\n", szIgnoreFile);
        while(fgets(aszIgnoreNetworks[iNumIgnoreNetworks], sizeof(aszIgnoreNetworks[iNumIgnoreNetworks]), pFile))
        {
          iNumIgnoreNetworks++;
          if(iNumIgnoreNetworks >= 16) break;
        }
        fclose(pFile);
      }
    }
  }
  /*****************************************************************************/
  
  //loop all interfaces
  S_InterfaceInfo* pstList = new S_InterfaceInfo[64];
  int i, iIfNum = 64;
  bool bGotInterfaces = C_Socket::GetLocalAddressInfo(pstList, &iIfNum);
  if(bGotInterfaces && iIfNum > 0)
  {
    for(i = 0; i < iIfNum; i++)
    {
      if(!pstList[i].bActive) continue; //this does not skip loopback If to support emulator
      //pstList[i].szBroadcast seems to be 255.255.255.255 always, and that is not what we want,
      // so we create the broadcast ip from ip and mask.
      int iSendToIP = C_Socket::ResolveAddress(pstList[i].szIP);
      int iMask = C_Socket::ResolveAddress(pstList[i].szMask);
      iSendToIP = (iSendToIP & iMask) | ~iMask;
      C_Socket::IPToString(iSendToIP, szText, sizeof(szText));
      /*****************************************************************************/
      //special ignore feature
      bool bIgnore = IsIgnoreNetwork(aszIgnoreNetworks, iNumIgnoreNetworks, szText);
      if(bIgnore)
      {
        gpLog->Print(LOG_TRACE, "[____] FZ_LL SendBroadcastToAllInterfaces: Ignoring %s broadcast on %s\n", pstData->iCommand == 0 ? "enumerate" : "reset ip cfg", szText);
        continue;
      }
      /*****************************************************************************/
      gpLog->Print(LOG_TRACE, "[____] FZ_LL SendBroadcastToAllInterfaces: Send %s broadcast on %s\n", pstData->iCommand == 0 ? "enumerate" : "reset ip cfg", szText);
      bool bBCastInit = pBcastSend->InitSendUDP(iSendToIP, iPort, true); //bcast to
      if(!bBCastInit)
      {
        gpLog->Print(LOG_ERROR, "[____] FZ_LL SendBroadcastToAllInterfaces: Send %s broadcast on %s, init failed\n", pstData->iCommand == 0 ? "enumerate" : "reset ip cfg", szText);
      }
      else pBcastSend->SendQueued((char*)pstData, iLen);
    }
  }
  else
  {
    //fallback to 255.255.255.255:
#define FALLBACK_BROADCAST_ADDR "255.255.255.255"
    gpLog->Print(LOG_WARN, "[____] FZ_LL SendBroadcastToAllInterfaces: Send %s broadcast, no If found, fallback to broadcast on %s\n", pstData->iCommand == 0 ? "enumerate" : "reset ip cfg", FALLBACK_BROADCAST_ADDR);
    int iSendToIP = C_Socket::ResolveAddress(FALLBACK_BROADCAST_ADDR);
    bool bBCastInit = pBcastSend->InitSendUDP(iSendToIP, iPort, true); //bcast to
    if(!bBCastInit)
    {
      gpLog->Print(LOG_ERROR, "[____] FZ_LL SendBroadcastToAllInterfaces: Send %s broadcast on %s, init failed\n", pstData->iCommand == 0 ? "enumerate" : "reset ip cfg", FALLBACK_BROADCAST_ADDR);
    }
    else pBcastSend->SendQueued((char*)pstData, iLen);
  }
  delete[] pstList;
  delete pBcastSend;
  
  //TODO: currently this function does not return errors
  return 0;
}

struct S_EthDevInfo
{
  int iIP;
  int iPort;
  char szSerial[16];
  int iDeviceType;
};
typedef std::vector<S_EthDevInfo> C_EthDeviceList;

#define PORT_RANGE_BASE 60000
#define PORT_RANGE_MAX  61999
C_EthDeviceList* FindEthDevices()
{
  char szText[128]; //debug text
  C_EthDeviceList* pList = new C_EthDeviceList();
  C_Socket* pUDPRecv = new C_Socket();
  //int iRecvPort = pUDPRecv->InitRecvUDP(); //uses port chosen by the OS, does not work well with firewalls.
  int iRecvPort, iTryPort = PORT_RANGE_BASE; //uses defined portrange and retries until a free port is found.
  while((iRecvPort = pUDPRecv->InitRecvUDP(iTryPort)) == -1 && iTryPort < 62000) iTryPort++;
  
  //Note: this does not apply any more since ports are now chosen in a defined range
  //avoid port FZ_BROADCASTPORT because we will get this and other machines enumeration
  // broadcasts to that port (intended for the camera), and broadcasts seems to be just
  // like any UDP packet on the receiver side, so it cant be filtered out here.
  if(iRecvPort == FZ_BROADCASTPORT)
  {
    C_Socket* pUDPRecvNew = new C_Socket();
    iRecvPort = pUDPRecvNew->InitRecvUDP();
    delete pUDPRecv;
    pUDPRecv = pUDPRecvNew;
    //we have avoided to receive on FZ_BROADCASTPORT
  }
  
  if(iRecvPort == -1)
  {
    gpLog->Print(LOG_ERROR, "[____] FZ_LL FindEthDevices: Send enumeration broadcast, init failed (UDP recv)\n");
    //TODO: currently this function does not return errors
    return pList; //empty
  }
  
  gpLog->Print(LOG_INFO, "[____] FZ_LL FindEthDevices: Send enumeration broadcast, expecting UDP reply on port %d\n", iRecvPort);
  //create packet to send
  FZ_LL_EthPCtoFZBroadCast stData;
  int iLen = sizeof(stData);
  stData.iSize = htonl(iLen);
  stData.szMagic[0] = 'F';
  stData.szMagic[1] = 'Z';
  stData.szMagic[2] = 'B';
  stData.szMagic[3] = '3';
  stData.iType = htonl(0);
  stData.iReplyPort = htonl(iRecvPort);
  stData.iCommand = 0;
  //send to all interfaces
  SendBroadcastToAllInterfaces(&stData, iLen, FZ_BROADCASTPORT);
  
  //collect reply packets (within timeout)
  char buf[SOCKET_BUFSIZE];
  int iIP, iPort;
  C_Timer* pTimer = new C_Timer();
  
  bool bGotPacket = false;
  do
  {
    iLen = -1; //indicate we want one packet regardless of size
    bGotPacket = pUDPRecv->GetRecvBuffer(buf, &iLen, 100, &iIP, &iPort);
    if(bGotPacket)
    {
      //handle reply packet
      
      //validate data
      FZ_LL_EthFZtoPCUDPReply* pstData2 = (FZ_LL_EthFZtoPCUDPReply*)buf;
      if(iLen == sizeof(FZ_LL_EthFZtoPCUDPReply))
      {
        if(strncmp(pstData2->szMagic, "FZB3", 4) == 0 && ntohl(pstData2->iSize) == (unsigned int)iLen &&
            (ntohl(pstData2->iType) == FZ_DEVICE_TYPE_JAGUAR_CA || ntohl(pstData2->iType) == FZ_DEVICE_TYPE_PANASONIC_CA || ntohl(pstData2->iType) == FZ_DEVICE_TYPE_PRIMESENSE_CA))
        {
          S_EthDevInfo stInfo;
          int iDeviceTypeCA = ntohl(pstData2->iType);
          if(iDeviceTypeCA == FZ_DEVICE_TYPE_PANASONIC_CA)       stInfo.iDeviceType = FZ_DEVICE_TYPE_PANASONIC;
          else if(iDeviceTypeCA == FZ_DEVICE_TYPE_PRIMESENSE_CA) stInfo.iDeviceType = FZ_DEVICE_TYPE_PRIMESENSE;
          else                                                 stInfo.iDeviceType = FZ_DEVICE_TYPE_JAGUAR;
          stInfo.iIP = ntohl(pstData2->iDeviceIP);
          stInfo.iPort = ntohl(pstData2->iDeviceListeningPort);
          strncpy(stInfo.szSerial, pstData2->szSerial, sizeof(stInfo.szSerial));
          stInfo.szSerial[sizeof(stInfo.szSerial) - 1] = 0;
          
          bool bDuplicateFound = false;
          int iSize = (int)pList->size();
          for(int j = 0; j < iSize; j++)
          {
            if(stInfo.iIP == (*pList)[j].iIP && stInfo.iPort == (*pList)[j].iPort)
            {
              bDuplicateFound = true;
              break;
            }
            //the comparison with port above is only to allow eth_sim to function
            // a real camera exist only once per ip
          }
          if(!bDuplicateFound)
          {
            pList->push_back(stInfo);
            gpLog->Print(LOG_TRACE, "[____] FZ_LL FindEthDevices: Got valid UDP reply on port %d from (%s), device listens on TCP port %d\n", iRecvPort, C_Socket::IPToString(stInfo.iIP, szText, sizeof(szText)), stInfo.iPort);
          }
        }
        else gpLog->Print(LOG_WARN, "[____] FZ_LL FindEthDevices: Got invalid UDP reply on port %d from (%s) (ok length but invalid data)\n", iRecvPort, C_Socket::IPToString(iIP, szText, sizeof(szText)));
      }
      else gpLog->Print(LOG_TRACE, "[____] FZ_LL FindEthDevices: Got invalid UDP reply on port %d from (%s) (invalid length %d, valid %d)\n", iRecvPort, C_Socket::IPToString(iIP, szText, sizeof(szText)), iLen, sizeof(FZ_LL_EthFZtoPCUDPReply));
    }
  }
  while(pTimer->PeekInterval() < 1000.0); //collected all replies during ~1000 ms
  
  delete pTimer;
  delete pUDPRecv;
  
  return pList;
}

bool EthernetGetDeviceInfo(C_EthDeviceList* pList, FZ_DEVICE_INFO* pDeviceInfo, int iIndex)
{
  int iSize = (int)pList->size();
  if(iIndex < iSize && iIndex >= 0)
  {
    S_EthDevInfo* pInfo = &((*pList)[iIndex]);
    strncpy(pDeviceInfo->szSerial, pInfo->szSerial, sizeof(pDeviceInfo->szSerial));
    pDeviceInfo->szSerial[sizeof(pDeviceInfo->szSerial) - 1] = 0;
    pDeviceInfo->iDeviceType = pInfo->iDeviceType;
    sprintf(pDeviceInfo->szPath, "%d.%d.%d.%d:%d", pInfo->iIP & 0xff, (pInfo->iIP >> 8) & 0xff, (pInfo->iIP >> 16) & 0xff, (pInfo->iIP >> 24) & 0xff, pInfo->iPort);
    sprintf(pDeviceInfo->szShortName, "%d.%d.%d.%d", pInfo->iIP & 0xff, (pInfo->iIP >> 8) & 0xff, (pInfo->iIP >> 16) & 0xff, (pInfo->iIP >> 24) & 0xff);
    return true;
  }
  return false;
}

FZ_Result EthOpenDevice(const char* szDevicePath, C_Socket** ppTCPCommand, C_Socket** ppTCPImage)
{
  FZ_Result iResult = FZ_DEVICE_NOT_FOUND;
  C_Socket* pTCPCommand = new C_Socket();
  C_Socket* pTCPImage = new C_Socket();
  
  char szIP[2048];
  int iPort = FZ_DEFAULT_COMMAND_TCPPORT;
  
  //string is a.b.c.d:port (port is optinal), a hostname should also be possible
  strncpy(szIP, szDevicePath, sizeof(szIP));
  szIP[sizeof(szIP) - 1] = 0;
  char* pLastColon = strrchr(szIP, ':');
  if(pLastColon)
  {
    //parse port number and remove from ip string
    sscanf(pLastColon + 1, "%d", &iPort);
    *pLastColon = 0; //nullterminate the string in the place of the last colon
  }
  
  //try to open command socket
  unsigned int iImagePort = 0;
  bool bConnect = pTCPCommand->Connect(szIP, iPort, 2000);
  if(bConnect)
  {
    int iLen = sizeof(iImagePort);
    if(pTCPCommand->GetRecvBuffer((char*)&iImagePort, &iLen, 2000))
    {
      //device opened, or busy
      if(iImagePort == 0)
      {
        iResult = FZ_DEVICE_BUSY;
        gpLog->Print(LOG_INFO, "[____] FZ_LL EthOpenDevice: (%s:%d) Replied busy\n", szIP, iPort);
      }
      else
      {
        iImagePort = ntohl(iImagePort); //device replies with port number for 2nd connection
        gpLog->Print(LOG_TRACE, "[____] FZ_LL EthOpenDevice: (%s:%d) Command socket connected, image socket port %d\n", szIP, iPort, iImagePort);
      }
    }
    else gpLog->Print(LOG_WARN, "[____] FZ_LL EthOpenDevice: (%s:%d) Command socket connected but no status received\n", szIP, iPort);
    //if connect reply timeout or busy iImagePort == 0 here
    // might need to send different errors later
  }
  else gpLog->Print(LOG_WARN, "[____] FZ_LL EthOpenDevice: (%s:%d) Command socket could not connect\n", szIP, iPort);
  
  if(iImagePort)
  {
    //try to open image socket
    bool bConnect = pTCPImage->Connect(szIP, iImagePort, 2000);
    if(bConnect)   //should always succeed
    {
      iResult = FZ_Success;
    }
    else gpLog->Print(LOG_WARN, "[____] FZ_LL EthOpenDevice: (%s:%d) Image socket could not connect on port %d\n", szIP, iPort, iImagePort);
  }
  
  //error:
  if(iResult == FZ_Success)
  {
    pTCPCommand->SetNoDelay();
    *ppTCPCommand = pTCPCommand;
    *ppTCPImage = pTCPImage;
  }
  else
  {
    delete pTCPCommand;
    delete pTCPImage;
  }
  return iResult;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//driver init/deinit

FZ_Result FZ_LL_Init()
{
  if(gbIsInitialized) return FZ_Success;
  
  if(!gbLLDrvCtxInitialized)
  {
    memset(&gLLDrvCtx, 0, sizeof(gLLDrvCtx)); //initialize driver context
    gLLDrvCtx.pMutex = new C_Mutex();
    gbLLDrvCtxInitialized = true;
  }
  
  //the mutex is needed to make sure init is done only once
  // if many threads are calling the DLL before it is initialized
  gLLDrvCtx.pMutex->Enter(); //lock
  
  //no special init required any more
  
  gLLDrvCtx.pMutex->Leave(); //unlock
  gpLog->Print(LOG_INFO, "[____] FZ_LL DriverInit: Done\n");
  return FZ_Success;
}

FZ_Result FZ_LL_Exit()
{
  if(!gbIsInitialized)
  {
    return FZ_NOT_INITIALIZED;
  }
  
  DRIVER_CONTEXT* pDrvCtx = &gLLDrvCtx;
  
  //free list of open devices
  // if open/close is used properly this list shall always be empty here
  FZ_Device_LL* pOpenDevice = pDrvCtx->pOpenDevices; //the first list element
  while(pOpenDevice)
  {
    printf("FZ_LL DriverCleanup -> Still open devices, closing\n");
    FZ_LL_Close(pOpenDevice->iDeviceNum); //this removes the list element
    pOpenDevice = pDrvCtx->pOpenDevices; //the now first list element
  }
  printf("FZ_LL DriverCleanup -> All closed\n");
  pDrvCtx->pOpenDevices = NULL;
  
  if(pDrvCtx->pMutex)
  {
    delete pDrvCtx->pMutex;
    pDrvCtx->pMutex = NULL;
  }
  
  return FZ_Success;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//enumerate

FZ_Result FZ_LL_GetSimpleDeviceName(
  char* szDevicePath,
  char* szShortName,
  int iShortNameLen)
{
  if(!szDevicePath || !szShortName || iShortNameLen < 16) return FZ_BAD_PARAMETERS;
  
  //simplify the device path into a shorter description string
  
  strncpy(szShortName, szDevicePath, iShortNameLen);
  szShortName[iShortNameLen - 1] = 0;
  char* szTmp = strrchr(szShortName, ':');
  if(szTmp) szTmp[0] = 0; //cut on last colon (to remove port for ip strings)
  
  return FZ_Success;
}

FZ_Result FZ_LL_EnumDevices(
  FZ_DEVICE_INFO* pDeviceInfo,
  int* piNumDevices)
{
  if(!gbIsInitialized)
  {
    return FZ_NOT_INITIALIZED;
  }
  
  //device context not needed, this function operates only on locals
  // so it is thread safe
  
  bool bFound;
  int i, iNumFound = 0;
  
  if(!piNumDevices || !pDeviceInfo || *piNumDevices <= 0)
    return FZ_BAD_PARAMETERS;
    
  int iMaxDevices = *piNumDevices;
  bool bNumFoundExceedsMaxAllowed = false;
  *piNumDevices = 0; //default report 0
  FZ_DEVICE_INFO stDefaultInfo;
  memset(&stDefaultInfo, 0, sizeof(stDefaultInfo));
  strncpy(stDefaultInfo.szSerial, "N/A", sizeof(stDefaultInfo.szSerial));
  stDefaultInfo.szSerial[sizeof(stDefaultInfo.szSerial) - 1] = 0;
  
  //find ethernet devices
  i = 0;
  C_EthDeviceList* pEthList = FindEthDevices(); //this takes min 1.2 sec
  do
  {
    FZ_DEVICE_INFO stNewInfo = stDefaultInfo;
    bFound = EthernetGetDeviceInfo(pEthList, &stNewInfo, i);
    if(bFound)
    {
      gpLog->Print(LOG_INFO, "[____] FZ_LL EnumDevices: found -> %s\n", stNewInfo.szPath);
      if(iNumFound + 1 > iMaxDevices)
      {
        bNumFoundExceedsMaxAllowed = true;
        break;
      }
      // set info struct
      pDeviceInfo[iNumFound] = stNewInfo;
      iNumFound++;
      i++;
    }
  }
  while(bFound);
  delete pEthList;
  
  //return result
  *piNumDevices = iNumFound;
  if(bNumFoundExceedsMaxAllowed)
  {
    return FZ_TOO_MANY_DEVICES;
  }
  
  return FZ_Success;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//open close

FZ_Device_LL* GetOpenDeviceStruct(FZ_Device_Handle_t hDev)
{
  FZ_Device_LL* pFirst, *pReturn = NULL;
  
  gLLDrvCtx.pMutex->Enter(); //lock
  for(pFirst = gLLDrvCtx.pOpenDevices; pFirst; pFirst = pFirst->pNext)
  {
    if(hDev == pFirst->iDeviceNum)
    {
      pReturn = pFirst;
      break;
    }
  }
  gLLDrvCtx.pMutex->Leave(); //unlock
  return pReturn;
}

bool SetCommandTimeout(FZ_Device_LL* pDev, int iValue)
{
  if(!pDev) return false;
  pDev->iCurrentCmdTimeout = (int)iValue;
  
  //this uses iCurrentCmdTimeout when waiting, no special set needed
  
  return true;
}

FZ_Result FZ_LL_Open(const char* szDevicePath, FZ_Device_Handle_t* phDev)
{
  if(!gbIsInitialized)
  {
    return FZ_NOT_INITIALIZED;
  }
  
  DRIVER_CONTEXT* pDrvCtx = &gLLDrvCtx;
  FZ_Device_LL* pNewOpenDevice = NULL;
  
  gpLog->Print(LOG_TRACE, "[____] FZ_LL Open: Begin device open (%s)\n", szDevicePath);
  
  if(!phDev || !szDevicePath) return FZ_BAD_PARAMETERS;
  
  FZ_Result iOpened = FZ_Failure;
  pNewOpenDevice = new FZ_Device_LL();
  memset(pNewOpenDevice, 0, sizeof(FZ_Device_LL));
  if(strchr(szDevicePath, '#'))   //USB device string
  {
    //USB no longer supported
    return FZ_NOT_SUPPORTED;
  }
  else
  {
    //try open on ethernet
    C_Socket* pTCPCommand = NULL, *pTCPImage = NULL;
    iOpened = EthOpenDevice(szDevicePath, &pTCPCommand, &pTCPImage);
    if(iOpened == FZ_Success)
    {
      //ethernet device opened
      pNewOpenDevice->pTCPCommand = pTCPCommand;
      pNewOpenDevice->pTCPImage = pTCPImage;
    }
  }
  if(iOpened == FZ_Success)
  {
    //common open procedure
    
    //add to open list (and find out which device number to assign)
    pDrvCtx->pMutex->Enter(); //lock
    FZ_Device_Handle_t iNextNum = 1;
    FZ_Device_LL** pOpenList = &pDrvCtx->pOpenDevices;
    while(*pOpenList)
    {
      FZ_Device_Handle_t iNum = (*pOpenList)->iDeviceNum;
      if(iNextNum <= iNum) iNextNum = iNum + 1;
      pOpenList = &((*pOpenList)->pNext);
    }
    strncpy(pNewOpenDevice->szDevicePath, szDevicePath, sizeof(pNewOpenDevice->szDevicePath));
    pNewOpenDevice->szDevicePath[sizeof(pNewOpenDevice->szDevicePath) - 1] = 0;
    *phDev = pNewOpenDevice->iDeviceNum = iNextNum;
    *pOpenList = pNewOpenDevice;
    pDrvCtx->pMutex->Leave(); //unlock
    
    if(InitializeStreamThread(pNewOpenDevice->iDeviceNum) != FZ_LL_SUCCESS)
    {
      FZ_LL_Close(*phDev);
      return FZ_Failure;
    }
    pNewOpenDevice->pCommandMutex = new C_Mutex();
  }
  else
  {
    //could not open
    delete pNewOpenDevice;
    int iLogLevel = LOG_ERROR;
    if(iOpened == FZ_DEVICE_BUSY) iLogLevel = LOG_WARN; //exclude busy as an error since it is normal
    gpLog->Print(iLogLevel, "[____] FZ_LL Open: Error opening device (%s) (code=%04x)\n", szDevicePath, iOpened);
    return iOpened;
  }
  
  //initialize the opened device
  SetCommandTimeout(pNewOpenDevice, 3500);
  
  gpLog->Print(LOG_INFO, "[%4x] FZ_LL Open: Device Opened %s\n", *phDev, szDevicePath);
  return FZ_Success;
}

FZ_Result FZ_LL_Close(FZ_Device_Handle_t hDev)
{
  if(!gbIsInitialized)
  {
    return FZ_NOT_INITIALIZED;
  }
  
  DRIVER_CONTEXT* pDrvCtx = &gLLDrvCtx;
  FZ_Device_LL* pDev = GetOpenDeviceStruct(hDev);
  
  gpLog->Print(LOG_TRACE, "[%4x] FZ_LL Close: Begin device close\n", hDev);
  
  if(!pDev) return FZ_BAD_HANDLE;
  
  //stop all PC activity (no stop command to camera is sent)
  StreamOff(hDev);
  
  pDev->bRemovalPending = true;
  pDev->pStreamThread->WaitForThreadExit();
  //^will deadlock if this function is called from DllMain, so it must not be called from there
  delete pDev->pStreamThread;
  delete pDev->pCommandMutex;
  
  pDrvCtx->pMutex->Enter(); //lock
  //remove from open list
  FZ_Device_LL** pOpenList = &pDrvCtx->pOpenDevices;
  while(*pOpenList)
  {
    if((*pOpenList)->iDeviceNum == hDev)
    {
      //remove this node
      FZ_Device_LL* pNext = (*pOpenList)->pNext;
      //ethernet
      delete(*pOpenList)->pTCPImage;
      delete(*pOpenList)->pTCPCommand;
      
      delete(*pOpenList);
      (*pOpenList) = pNext;
      break;
    }
    pOpenList = &((*pOpenList)->pNext);
  }
  pDrvCtx->pMutex->Leave(); //unlock
  
  gpLog->Print(LOG_INFO, "[%4x] FZ_LL Close: Device closed\n", hDev);
  return FZ_Success;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//other user calls

#define DEBUG_COMMANDS

FZ_Result FZ_LL_SendCommand(
  FZ_Device_Handle_t hDev,    //device handle
  unsigned short iCmd,        //command code
  unsigned char* pParam,      //pointer to parameter
  int iParamSize,             //paramter size (bytes)
  unsigned short* piRespCode, //pointer to response code
  unsigned char* pRespParam,  //pointer to response parameter
  int* piRespSize,            //pointer to response parameter size
  unsigned short iOffset,
  unsigned int iReturnBytesRequested)
{
  if(!gbIsInitialized)
  {
    return FZ_NOT_INITIALIZED;
  }
  
  static unsigned int iCmdNo = 0; //ever increasing cmd identifier
  FZ_LL_Pkt_t cmdpkt, rsppkt;
  FZ_Result iReturn = FZ_Success;
  
  FZ_Device_LL* pDev = GetOpenDeviceStruct(hDev);
  
  //error checking
  if(!pDev)
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_LL SendCommand: No device\n", hDev);
    return FZ_BAD_HANDLE;
  }
  if(iParamSize > FZ_LL_PARAMETER_MAX_SIZE)
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_LL SendCommand: iParamSize>FZ_LL_PARAMETER_MAX_SIZE (%d)!\n", hDev, FZ_LL_PARAMETER_MAX_SIZE);
    return FZ_BAD_PARAMETERS;
  }
  if(iParamSize && !pParam)
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_LL SendCommand: iParamSize>0, but data pointer is NULL!\n", hDev);
    return FZ_BAD_PARAMETERS;
  }
  
  cmdpkt.serialn = ++iCmdNo;
  cmdpkt.code = iCmd;
  cmdpkt.bytelen = (uint16_t)iParamSize;
  cmdpkt.offset = iOffset;
  cmdpkt.iReturnBytesRequested = iReturnBytesRequested;
  if(pParam) memcpy(cmdpkt.prm, pParam, iParamSize);
  else memset(cmdpkt.prm, 0, sizeof(cmdpkt.prm));
  
  pDev->pCommandMutex->Enter();
  
  //if a stop command is seen - stop the stream thread
  if(cmdpkt.code == CMD_DE_SENSOR_STOP)
  {
    StreamOff(hDev);
  }
  //if a start command is seen - start the stream thread
  if(cmdpkt.code == CMD_DE_SENSOR_START)
  {
    StreamOn(hDev);
  }
  
  //if a command that takes a long time to complete is seen - set longer timeout
  if(cmdpkt.code == CMD_DE_BURN_FLASH_CFG)      SetCommandTimeout(pDev, 260 * 1000);
  if(cmdpkt.code == CMD_DE_STORE_LOOKUP_TABLES) SetCommandTimeout(pDev, 120 * 1000);
  if(cmdpkt.code == CMD_DE_UPDATE_PROGRAM_CA || cmdpkt.code == CMD_DE_UPDATE_PROGRAM_DE) SetCommandTimeout(pDev, 15 * 1000);
  
#ifdef DEBUG_COMMANDS
  gpLog->Print(LOG_TRACE, "[%4x] FZ_LL SendCommand: Cmd 0x%04x, first short in param 0x%04x\n", hDev, cmdpkt.code, *((unsigned short*)&cmdpkt.prm[0]));
#endif
  {
    //ethernet
    FZ_LL_EthCommand pkt;
    C_Socket* pCmdSocket = pDev->pTCPCommand;
    if(!pCmdSocket)
    {
      gpLog->Print(LOG_ERROR, "[%4x] FZ_LL SendCommand (eth): No handle (shall never happen)\n", hDev);
      iReturn = FZ_BAD_HANDLE;
      goto error;
    }
    int iLen = sizeof(pkt);
    pkt.szMagic[0] = 'F';
    pkt.szMagic[1] = 'Z';
    pkt.szMagic[2] = 'B';
    pkt.szMagic[3] = '3';
    pkt.iSize = htonl(iLen);
    pkt.iType = htonl(2);
    pkt.stCmd = cmdpkt;
    
    //to handle sync problems:
    //if for example the last command sent got no reply within timeout, but the socket
    // survived and got the reply later, this command will get the older commands reply.
    //to handle it we compare a number on sent and received command packets that must match
    uint32_t iSentSerial = pkt.stCmd.serialn;
    
    if(iLen != pCmdSocket->SendQueued((char*)&pkt, iLen))
    {
      gpLog->Print(LOG_ERROR, "[%4x] FZ_LL SendCommand (eth): Error sending cmd\n", hDev);
      if(pCmdSocket->SendBufferSize() > 0)
      {
        //clear this command from buffer since we have failed the send
        //the serialnumber handling described above is still nessecary because
        // commands may have left the queue and will be sent by the OS when the
        // remote side responds
        pCmdSocket->ClearSendQueue();
      }
      iReturn = FZ_CMD_SEND_FAILED;
      goto error;
    }
    do
    {
      iLen = sizeof(pkt);
      if(!pCmdSocket->GetRecvBuffer((char*)&pkt, &iLen, pDev->iCurrentCmdTimeout))
      {
        gpLog->Print(LOG_ERROR, "[%4x] FZ_LL SendCommand (eth): Error reading response\n", hDev);
        iReturn = FZ_CMD_RECV_FAILED;
        goto error;
      }
      
      if(strncmp(pkt.szMagic, "FZB3", 4) != 0 ||
          (ntohl(pkt.iSize) != sizeof(pkt)) || (iLen != sizeof(pkt)) ||
          ntohl(pkt.iType) != 2)
      {
        gpLog->Print(LOG_ERROR, "[%4x] FZ_LL SendCommand (eth): Error in response\n", hDev);
        iReturn = FZ_CMD_RECV_FAILED;
        goto error;
      }
      if(iSentSerial != pkt.stCmd.serialn)
      {
        gpLog->Print(LOG_WARN, "[%4x] FZ_LL SendCommand (eth): Read command response mismatch, read again\n", hDev);
      }
    }
    while(iSentSerial != pkt.stCmd.serialn);
    rsppkt = pkt.stCmd;
  }
#ifdef DEBUG_COMMANDS
  gpLog->Print(LOG_TRACE, "[%4x] FZ_LL SendCommand: Got back value 0x%04x\n", hDev, rsppkt.code);
#endif
  
  if(piRespCode)
  {
    *piRespCode = rsppkt.code;
    if(piRespSize)
    {
      *piRespSize = rsppkt.bytelen;
      memcpy(pRespParam, rsppkt.prm, rsppkt.bytelen);
    }
  }
  
  iReturn = FZ_Success;
error:
  //always set the command timeouts to the default here
  SetCommandTimeout(pDev, 3500);
  pDev->pCommandMutex->Leave();
  
  return iReturn;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
