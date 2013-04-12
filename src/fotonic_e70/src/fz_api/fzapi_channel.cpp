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

#include "fzapi_channel.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "fz_internal.h"
#include "fz_api.h"

#include "logger.h"

#define CHANNEL_RECV_PORT_BASE 5000

extern volatile bool gbIsInitialized;                 //from fzapi.cpp
extern C_Logger* gpLog;                               //from fzapi.cpp
extern C_Mutex* gpMutex2;                             //from fzapi.cpp
extern FZ_Channel gChannels[MAX_CHANNELS];            //from fzapi.cpp

FZ_API FZ_Result FZ_OpenFrameChannel(
  int iChannel,
  int iFlags)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  gpLog->Print(LOG_INFO, "[____] FZ_API FZ_OpenFrameChannel[%d]: Flags 0x%02x\n", iChannel, iFlags);
  
  if(iChannel > MAX_CHANNELS || iChannel < 0)
  {
    gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_OpenFrameChannel[%d]: Bad channel\n", iChannel);
    return FZ_BAD_HANDLE;
  }
  
  gpMutex2->Enter();
  if(gChannels[iChannel].iState != 0) //not closed
  {
    gpMutex2->Leave();
    gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_OpenFrameChannel[%d]: Channel busy\n", iChannel);
    return FZ_DEVICE_BUSY;
  }
  
  if(iFlags & FZ_FRAME_CHAN_SEND)
  {
    //connect
    C_Socket* pclSocket = new C_Socket();
    gChannels[iChannel].iState = 2;
    gpMutex2->Leave();
    bool bConnected = pclSocket->Connect("127.0.0.1", CHANNEL_RECV_PORT_BASE + iChannel, 5000);
    if(bConnected)
    {
      //pclSocket->SetNoDelay();
      gChannels[iChannel].pclSocket = pclSocket;
      gChannels[iChannel].iState = 4;
      gpLog->Print(LOG_TRACE, "[____] FZ_API FZ_OpenFrameChannel[%d]: Send connection OK\n", iChannel);
      return FZ_Success;
    }
    else
    {
      delete pclSocket;
    }
  }
  else if(iFlags & FZ_FRAME_CHAN_RECV)
  {
    //listen and accept
    C_Socket* pclListenSocket = new C_Socket(CHANNEL_RECV_PORT_BASE + iChannel);
    C_Socket* pclSocket = NULL;
    gChannels[iChannel].iState = 1;
    gpMutex2->Leave();
    
    for(int i = 0; i < 10; i++)
    {
      pclListenSocket->WaitForReadEvent(1000);
      if(pclListenSocket->m_bAcceptWillSucceed)
      {
        pclSocket = pclListenSocket->AcceptConnection();
        break;
      }
    }
    
    delete pclListenSocket;
    if(pclSocket)
    {
      gChannels[iChannel].pclSocket = pclSocket;
      gChannels[iChannel].iState = 3;
      gpLog->Print(LOG_TRACE, "[____] FZ_API FZ_OpenFrameChannel[%d]: Recv connection OK\n", iChannel);
      return FZ_Success;
    }
  }
  else
  {
    gpMutex2->Leave();
  }
  gChannels[iChannel].iState = 0;
  gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_OpenFrameChannel[%d]: Timeout or other socket error\n", iChannel);
  return FZ_TIMEOUT;
}

FZ_API FZ_Result FZ_CloseFrameChannel(
  int iChannel)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  gpLog->Print(LOG_INFO, "[____] FZ_API FZ_CloseFrameChannel[%d]\n", iChannel);
  
  if(iChannel > MAX_CHANNELS || iChannel < 0)
  {
    gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_CloseFrameChannel[%d]: Bad channel\n", iChannel);
    return FZ_BAD_HANDLE;
  }
  
  if(gChannels[iChannel].iState == 0) //already closed
  {
    gpLog->Print(LOG_INFO, "[____] FZ_API FZ_CloseFrameChannel[%d]: Not open\n", iChannel);
    return FZ_Success;
  }
  if(gChannels[iChannel].iState == 1 || gChannels[iChannel].iState == 2) //in a connecting state
  {
    gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_CloseFrameChannel[%d]: In a connecting state\n", iChannel);
    return FZ_NOT_SUPPORTED;
  }
  
  gpMutex2->Enter();
  delete gChannels[iChannel].pclSocket;
  gChannels[iChannel].pclSocket = NULL;
  gChannels[iChannel].iState = 0;
  gpMutex2->Leave();
  return FZ_Success;
}

FZ_API FZ_Result FZ_SendFrameToChannel(
  int iChannel,
  FZ_FRAME_HEADER* pHeader,
  void* pPixels)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  if(iChannel > MAX_CHANNELS || iChannel < 0)
  {
    gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_SendFrameToChannel[%d]: Bad channel\n", iChannel);
    return FZ_BAD_HANDLE;
  }
  if(!pHeader || !pPixels)
  {
    gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_SendFrameToChannel[%d]: Bad parameters\n", iChannel);
    return FZ_BAD_PARAMETERS;
  }
  if(gChannels[iChannel].iState != 4) //not open to send
  {
    gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_SendFrameToChannel[%d]: Not open to send\n", iChannel);
    return FZ_NOT_SUPPORTED;
  }
  
  C_Socket* pclSocket = gChannels[iChannel].pclSocket;
  
  int iRet, iLen = pHeader->ncols * pHeader->nrows * pHeader->bytesperpixel;
  pclSocket->WaitForWriteEvent(0);
  pclSocket->SendQueued((char*)pHeader, sizeof(FZ_FRAME_HEADER));
  iRet = pclSocket->SendQueued((char*)pPixels, iLen);
  if(iRet == iLen)
  {
    gpLog->Print(LOG_TRACE, "[____] FZ_API FZ_SendFrameToChannel[%d]: Send frame OK\n", iChannel);
    return FZ_Success;
  }
  gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_SendFrameToChannel[%d]: Send frame failed\n", iChannel);
  return FZ_Failure;
}

FZ_API FZ_Result FZ_GetFrameFromChannel(
  int iChannel,
  FZ_FRAME_HEADER* pHeader,
  void* pPixels, size_t* piPixelsByteLen)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  if(iChannel > MAX_CHANNELS || iChannel < 0)
  {
    gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_GetFrameFromChannel[%d]: Bad channel\n", iChannel);
    return FZ_BAD_HANDLE;
  }
  if(!pHeader || !pPixels || !piPixelsByteLen)
  {
    gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_GetFrameFromChannel[%d]: Bad parameters\n", iChannel);
    return FZ_BAD_PARAMETERS;
  }
  if(gChannels[iChannel].iState != 3) //not open to recv
  {
    gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_GetFrameFromChannel[%d]: Not open to recv\n", iChannel);
    return FZ_NOT_SUPPORTED;
  }
  
  C_Socket* pclSocket = gChannels[iChannel].pclSocket;
  pclSocket->WaitForReadEvent(0);
  
  bool bRet;
  int iLen = sizeof(FZ_FRAME_HEADER);
  int iRet = iLen;
  bRet = pclSocket->GetRecvBuffer((char*)pHeader, &iRet, 5000);
  if(bRet && iRet == iLen)
  {
    iRet = iLen = pHeader->ncols * pHeader->nrows * pHeader->bytesperpixel;
    if(*piPixelsByteLen < (size_t)iLen)
    {
      gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_GetFrameFromChannel[%d]: Too small buffer (%d, need %d)\n", iChannel, *piPixelsByteLen, iLen);
      bRet = pclSocket->GetRecvBuffer(NULL, &iRet, 5000); //waste data to keep in sync
      return FZ_BAD_PARAMETERS;
    }
    else
    {
      bRet = pclSocket->GetRecvBuffer((char*)pPixels, &iRet, 5000);
      if(bRet && iRet == iLen)
      {
        *piPixelsByteLen = iLen;
        gpLog->Print(LOG_TRACE, "[____] FZ_API FZ_GetFrameFromChannel[%d]: Got frame\n", iChannel);
        return FZ_Success;
      }
    }
  }
  gpLog->Print(LOG_ERROR, "[____] FZ_API FZ_GetFrameFromChannel[%d]: Failed to get frame\n", iChannel);
  return FZ_Failure;
}
