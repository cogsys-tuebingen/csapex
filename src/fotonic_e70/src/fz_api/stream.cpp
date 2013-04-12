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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "fz_ll_internal.h"
#include "fz_ll.h"

#include "logger.h"

extern volatile bool  gbIsInitialized; //from fzapi.cpp
extern C_Logger*       gpLog;    //from fz_api.cpp
extern DRIVER_CONTEXT gLLDrvCtx; //from fz_ll.cpp

THREAD_RET StreamThread(void* pParameter);

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
// implementation of an atomic queue

// see stream.h for variables

static int StreamQueuePop(FZ_Device_LL* pDev, bool iTakeLock = true)
{
  int iRes;
  if(pDev->iQueueSize <= 0)
  {
    //printf("pop: starved!\n");
    return -1;
  }
  
  if(iTakeLock)   //used because the C_Mutex does not use a counting mutex in the linux impl
  {
    pDev->pQueueMutex->Enter(); //lock
  }
  
  iRes = pDev->aiQueueImgnr[pDev->iQueuePopPos];
  pDev->iQueuePopPos++;
  pDev->iQueuePopPos %= QUEUE_BUFFERS;
  pDev->iQueueSize--;
  //printf("pop: %d\n", iRes);
  
  if(iTakeLock)
  {
    pDev->pQueueMutex->Leave(); //unlock
  }
  return iRes;
}

static int StreamQueuePush(FZ_Device_LL* pDev, int iNr)
{
  int iRes = 0;
  pDev->pQueueMutex->Enter(); //lock
  if(pDev->iQueueSize >= QUEUE_BUFFERS)
  {
    //if the queue size will exceed the max allowed
    // use StreamQueuePop() to waste oldest then push
    //printf("push: %d overflow!\n", iNr);
    StreamQueuePop(pDev, false);
  }
  //printf("push: %d\n", iNr);
  pDev->aiQueueImgnr[pDev->iQueuePushPos] = iNr;
  pDev->iQueuePushPos++;
  pDev->iQueuePushPos %= QUEUE_BUFFERS;
  pDev->iQueueSize++;
  pDev->pQueueMutex->Leave(); //unlock
  return iRes;
}

static void StreamQueueReset(FZ_Device_LL* pDev)
{
  pDev->pQueueMutex->Enter(); //lock
  pDev->iQueuePopPos  = 0;
  pDev->iQueuePushPos = 0;
  pDev->iQueueSize    = 0;
  pDev->pQueueMutex->Leave(); //unlock
}
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

int InitializeStreamThread(FZ_Device_Handle_t hDev)
{
  FZ_Device_LL* pDev = GetOpenDeviceStruct(hDev);
  
  gpLog->Print(LOG_TRACE, "[%4x] FZ_LL InitializeStreamThread: Begin\n", hDev);
  
  pDev->pQueueMutex = new C_Mutex();
  StreamQueueReset(pDev);
  pDev->currdrvbuf = 0;
  
  pDev->pFrameReady = new C_Event();
  pDev->pStreamOn   = new C_Event();
  pDev->pStreamOff  = new C_Event();
  if(!pDev->pFrameReady || !pDev->pStreamOn || !pDev->pStreamOff)
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_LL InitializeStreamThread: Error creating events\n", hDev);
    return FZ_LL_FAIL;
  }
  
  pDev->pStreamThread = new C_Thread(StreamThread, (void*)(intptr_t)hDev);
  gpLog->Print(LOG_INFO, "[%4x] FZ_LL InitializeStreamThread: Done\n", hDev);
  
  return FZ_LL_SUCCESS;
}

int StreamOn(FZ_Device_Handle_t hDev)
{
  FZ_Device_LL* pDev = GetOpenDeviceStruct(hDev);
  
  if(pDev->bStreamRunning)
    return FZ_LL_SUCCESS;
    
  gpLog->Print(LOG_TRACE, "[%4x] FZ_LL StreamOn: Begin\n", hDev);
  //reset the stream frame queue
  StreamQueueReset(pDev);
  
  if(pDev->pStreamOn)
  {
    pDev->pStreamOn->Signal();
    
    while(!pDev->bStreamRunning)
      mssleep(1);
      
    gpLog->Print(LOG_INFO, "[%4x] FZ_LL StreamOn: Done\n", hDev);
    return FZ_LL_SUCCESS;
  }
  else
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_LL StreamOn: No event handle\n", hDev);
    return FZ_LL_FAIL;
  }
}

int StreamOff(FZ_Device_Handle_t hDev)
{
  FZ_Device_LL* pDev = GetOpenDeviceStruct(hDev);
  
  if(!pDev->bStreamRunning)
    return FZ_LL_SUCCESS;
    
  gpLog->Print(LOG_TRACE, "[%4x] FZ_LL StreamOff: Begin\n", hDev);
  if(pDev->pStreamOff)
  {
    pDev->pStreamOff->Signal();
    
    while(pDev->bStreamRunning)
    {
      //let other thread run
      mssleep(1);
    }
    
    gpLog->Print(LOG_INFO, "[%4x] FZ_LL StreamOff: Done\n", hDev);
    return FZ_LL_SUCCESS;
  }
  else
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_LL StreamOff: No event handle\n", hDev);
    return FZ_LL_FAIL;
  }
}

FZ_Result FZ_LL_FrameAvailable(FZ_Device_Handle_t hDev)
{
  if(!gbIsInitialized)
  {
    return FZ_NOT_INITIALIZED;
  }
  
  FZ_Device_LL* pDev = GetOpenDeviceStruct(hDev);
  
  if(!pDev)
    return FZ_BAD_HANDLE;
  if(!pDev->bStreamRunning)
    return FZ_STREAM_NOT_RUNNING;
  if(pDev->iQueueSize > 0)
    return FZ_Success;
  return FZ_STREAM_NO_IMAGES;
}

////////////////////////////////////////////////////////////////////////////////////////////
// wait for and copy an image to the caller
////////////////////////////////////////////////////////////////////////////////////////////
int16_t GetRadialZFromBZXY(int z, int x, int y)
{
  double dz = x;
  double dx = y;
  double dy = z;
  
  double length = sqrt(dx * dx + dy * dy + dz * dz);
  return (int16_t)length;
}

// maps unsigned 8 bits/channel to DWORD rgb
#define COLOR_ARGB(a,r,g,b) \
  ((uint32_t)((((a)&0xff)<<24)|(((r)&0xff)<<16)|(((g)&0xff)<<8)|((b)&0xff)))
#define COLOR_RGBA(r,g,b,a) COLOR_ARGB(a,r,g,b)
#define COLOR_XRGB(r,g,b)   COLOR_ARGB(0xff,r,g,b)

// figure out the RGB values of B based on pixelformat
void GetRGBFromBZXY(uint32_t* pOutput, short* pPixels, int pixelformat, bool bOdd, int maxB)
{
  if(pixelformat == FZ_PIXELFORMAT_YUV422)
  {
    // YUV422
    if(bOdd) pPixels--;
    FZ_YUV422_DOUBLE_PIXEL* p = (FZ_YUV422_DOUBLE_PIXEL*)pPixels;
    
    float r, g, b, u, v, u1, v1, uv1;
    
    u = p->u - 128.0f;
    v = p->v - 128.0f;
    v1 = 1.13983f * v;
    uv1 = -0.39465f * u - 0.58060f * v;
    u1 = 0.03211f * u;
    if(!bOdd)
    {
    
      r = p->y1 + v1;
      g = p->y1 + uv1;
      b = p->y1 + u1;
      
      r = min(255, max(0, r));
      g = min(255, max(0, g));
      b = min(255, max(0, b));
      
      *pOutput = COLOR_XRGB(unsigned(r), unsigned(g), unsigned(b));
    }
    else
    {
      r = p->y2 + v1;
      g = p->y2 + uv1;
      b = p->y2 + u1;
      
      r = min(255, max(0, r));
      g = min(255, max(0, g));
      b = min(255, max(0, b));
      
      *pOutput = COLOR_XRGB(unsigned(r), unsigned(g), unsigned(b));
    }
  }
  else
  {
    //B16
    float b = *pPixels;
    b = (b / maxB) * 255.0f;
    b = min(255, max(0, b));
    *pOutput = COLOR_XRGB(unsigned(b), unsigned(b), unsigned(b));
  }
}

FZ_Result FZ_LL_GetFrame(FZ_Device_Handle_t hDev, void* pHeader, void* pPixels, int iPixelsByteLen, int x, int y, int w, int h, int iFlags)
{
  FZ_Device_LL* pDev;
  int iCurrframe;
  
  if(!gbIsInitialized)
  {
    return FZ_NOT_INITIALIZED;
  }
  
restart:
  pDev = GetOpenDeviceStruct(hDev);
  if(!pDev)
    return FZ_BAD_HANDLE;
  if(!pDev->bStreamRunning)
    return FZ_STREAM_NOT_RUNNING;
    
  //get one frame from the queue
  
  //wait for frame impl
  iCurrframe = StreamQueuePop(pDev);
  if(iCurrframe < 0)
  {
    if(pDev->pFrameReady->Wait(1200)) goto restart;
    gpLog->Print(LOG_WARN, "[%4x] FZ_LL GetFrame: Timeout\n", hDev);
    return FZ_TIMEOUT;
  }
  
  //copy currframe to buffer
  // raw mode is also returned as 8 bytes per pixel, x,y invalid
  //supports new feature to repack camera fmt to any user fmt set with FZ_SetFrameDataFmt
  
  //set processing
  bool bMirror = (iFlags & FZ_FMT_PROCESS_MIRROR) != 0;
  bool bInterleaved = (iFlags & FZ_FMT_PIXEL_INTERLEAVED) != 0;
  //check buffer size
  int iDstBytesPerPixel = 0;
  if((iFlags & FZ_FMT_COMPONENT_B) != 0)       iDstBytesPerPixel += 2;
  if((iFlags & FZ_FMT_COMPONENT_Z) != 0)       iDstBytesPerPixel += 2;
  if((iFlags & FZ_FMT_COMPONENT_XY) != 0)      iDstBytesPerPixel += 2 * 2;
  if((iFlags & FZ_FMT_COMPONENT_RADIALZ) != 0) iDstBytesPerPixel += 2;
  if((iFlags & FZ_FMT_COMPONENT_RGB) != 0)     iDstBytesPerPixel += 4;
  int ncols = pDev->dheadq[iCurrframe].ncols;
  int nrows = pDev->dheadq[iCurrframe].nrows;
  int fmt = pDev->dheadq[iCurrframe].pixelformat;
  
  if(w <= 0 || h <= 0)
  {
    if(iPixelsByteLen < ncols * nrows * iDstBytesPerPixel) return FZ_BAD_PARAMETERS;
    w = ncols;
    h = nrows;
    x = 0;
    y = 0;
  }
  else
  {
    if(iPixelsByteLen < w * h * iDstBytesPerPixel) return FZ_BAD_PARAMETERS;
  }
  
  //copy header
  memcpy(pHeader, &pDev->dheadq[iCurrframe], sizeof(FZ_LL_FRAME_HEADER));
  ((FZ_LL_FRAME_HEADER*)pHeader)->bytesperpixel = (uint16_t)iDstBytesPerPixel; //patch pixel size
  //copy data and reorder to user fmt
  int iDstRowBytes = iDstBytesPerPixel * w;
  int iSrcPixelBytes = 8; //always 8
  int iSrcRowBytes = iSrcPixelBytes * ncols;
  int iSrcPixelStartOffset = 0;
  int iDstComponentB_AddW, iDstComponentZ_AddW;
  int iDstComponentX_AddW, iDstComponentY_AddW;
  int iDstComponentRZ_AddW = 0;
  int iDstComponentRGB_AddW = 0;
  int iSrcComponentB_Add   = ncols * 0;
  int iSrcComponentZ_Add   = ncols * 2;
  int iSrcComponentX_Add   = ncols * 4;
  int iSrcComponentY_Add   = ncols * 6;
  int iAdd = 0;
  int iPosComponentMult = 1;
  //count component size
  iDstComponentB_AddW = 0;
  if((iFlags & FZ_FMT_COMPONENT_B) != 0)       iAdd += 1;
  iDstComponentZ_AddW = iAdd;
  if((iFlags & FZ_FMT_COMPONENT_Z) != 0)       iAdd += 1;
  iDstComponentX_AddW = iAdd;
  iDstComponentY_AddW = iAdd + 1;
  if((iFlags & FZ_FMT_COMPONENT_XY) != 0)      iAdd += 2;
  iDstComponentRZ_AddW = iAdd;
  if((iFlags & FZ_FMT_COMPONENT_RADIALZ) != 0) iAdd += 1;
  iDstComponentRGB_AddW = iAdd;
  if(!bInterleaved)
  {
    iDstComponentB_AddW  *= w;
    iDstComponentZ_AddW  *= w;
    iDstComponentX_AddW  *= w;
    iDstComponentY_AddW  *= w;
    iDstComponentRZ_AddW *= w;
    iDstComponentRGB_AddW *= w;
  }
  if(bMirror)
  {
    iSrcPixelBytes = -iSrcPixelBytes;
    iSrcPixelStartOffset = (iSrcRowBytes + iSrcPixelBytes) / 4; //last pixel on first plane
  }
  union
  {
    uint8_t* pDstB;
    int16_t* pDstW;
  };
  pDstB = (uint8_t*)pPixels;
  uint8_t* pSrcB = pDev->dbufq[iCurrframe];
  for(int j = y; j < y + h; j++)
  {
    //each dst row
    if(j < 0 || j >= nrows)
    {
      //set all data on row to 0
      memset(pDstB, 0, iDstRowBytes);
    }
    else
    {
      uint8_t* pRowSrcB = pSrcB + iSrcPixelStartOffset;
      if(x > 0) pRowSrcB += x * (iSrcPixelBytes / 4);
      for(int i = x; i < x + w; i++)
      {
        //each pixel on dst row
        if(bInterleaved)
        {
          iPosComponentMult = i;
          iAdd = 0;
        }
        else
        {
          iAdd = i;
        }
        //build result pixel
        if(i < 0 || i >= ncols)
        {
          //set all data on pixel to 0
          if((iFlags & FZ_FMT_COMPONENT_B) != 0)
          {
            pDstW[iDstComponentB_AddW * iPosComponentMult + iAdd] = 0;
          }
          if((iFlags & FZ_FMT_COMPONENT_Z) != 0)
          {
            pDstW[iDstComponentZ_AddW * iPosComponentMult + iAdd] = 0;
          }
          if((iFlags & FZ_FMT_COMPONENT_XY) != 0)
          {
            pDstW[iDstComponentX_AddW * iPosComponentMult + iAdd] = 0;
            pDstW[iDstComponentY_AddW * iPosComponentMult + iAdd] = 0;
          }
          if((iFlags & FZ_FMT_COMPONENT_RADIALZ) != 0)
          {
            pDstW[iDstComponentRZ_AddW * iPosComponentMult + iAdd] = 0;
          }
          if((iFlags & FZ_FMT_COMPONENT_RGB) != 0)
          {
            int wpos = iDstComponentRGB_AddW * iPosComponentMult + iAdd;
            pDstW[wpos] = 0;
            pDstW[wpos + 1] = 0;
          }
        }
        else
        {
          if((iFlags & FZ_FMT_COMPONENT_B) != 0)
          {
            pDstW[iDstComponentB_AddW * iPosComponentMult + iAdd] = *((int16_t*)(pRowSrcB + iSrcComponentB_Add));
          }
          if((iFlags & FZ_FMT_COMPONENT_Z) != 0)
          {
            pDstW[iDstComponentZ_AddW * iPosComponentMult + iAdd] = *((int16_t*)(pRowSrcB + iSrcComponentZ_Add));
          }
          if((iFlags & FZ_FMT_COMPONENT_XY) != 0)
          {
            pDstW[iDstComponentX_AddW * iPosComponentMult + iAdd] = *((int16_t*)(pRowSrcB + iSrcComponentX_Add));
            int16_t Y = *((int16_t*)(pRowSrcB + iSrcComponentY_Add));
            if((iFlags & FZ_FMT_PROCESS_INVERTY) != 0) Y *= -1;
            pDstW[iDstComponentY_AddW * iPosComponentMult + iAdd] = Y;
          }
          if((iFlags & FZ_FMT_COMPONENT_RADIALZ) != 0)
          {
            pDstW[iDstComponentRZ_AddW * iPosComponentMult + iAdd] = GetRadialZFromBZXY(
                  *(int16_t*)(pRowSrcB + iSrcComponentZ_Add), *(int16_t*)(pRowSrcB + iSrcComponentX_Add), *(int16_t*)(pRowSrcB + iSrcComponentY_Add));
          }
          if((iFlags & FZ_FMT_COMPONENT_RGB) != 0)
          {
            int wpos = iDstComponentRGB_AddW * iPosComponentMult + iAdd;
            
            //TODO: now we scale to 1024 since we dont know the device type here (device type 0 has max 2650, device type 1 has 1024, 2 has YUV and dont use max)
            GetRGBFromBZXY((uint32_t*)(&pDstW[wpos]), ((int16_t*)(pRowSrcB + iSrcComponentB_Add)), fmt, (i & 1) != 0, 1024);
          }
          
          pRowSrcB += iSrcPixelBytes / 4;
        }
      }
      pSrcB += iSrcRowBytes;
    }
    pDstB += iDstRowBytes;
  }
  
  return FZ_Success;
}

int ScanForEthImgHeader(char* pData, int iLen) //returns found position or -100
{
  int i;
  union
  {
    int iPattern;
    char szPattern[4];
  };
  szPattern[0] = 'F';
  szPattern[1] = 'Z';
  szPattern[2] = 'B';
  szPattern[3] = '3';
  for(i = 0; i < iLen - 4; i++)
  {
    //scan for "FZB3" and then test for image header
    int* p = (int*)(pData + i);
    if(*p == iPattern)
    {
      return i - 4;
    }
  }
  return -100;
}

////////////////////////////////////////////////////////////////////////////////////////////
// thread: used to process the incoming data from the sensor
////////////////////////////////////////////////////////////////////////////////////////////
THREAD_RET StreamThread(void* pParameter)
{
  unsigned int iLastFrame = 0;
  bool bFirstFrame = true;
  int iAction;
  FZ_Device_Handle_t hDev = (FZ_Device_Handle_t)((intptr_t)pParameter);
  FZ_Device_LL* pDev = GetOpenDeviceStruct(hDev);
  time_t tv;
  int iCurrBufNr;
  int iSleep;
  bool bDeadSocketReported = false;
  
  if(!pDev->pStreamOn || !pDev->pStreamOff)
  {
    delete pDev->pStreamOn;
    pDev->pStreamOn = NULL;
    delete pDev->pStreamOff;
    pDev->pStreamOff = NULL;
    return (THREAD_RET_TYPE)FZ_LL_FAIL;
  }
  
  while(!(pDev->bRemovalPending))
  {
    iSleep = 0;
    //gDrvCtx.hMutex need not be locked by this thread since we always wait
    // for thread to stop before closing the device, and for stream to stop
    // before any other operation that accesses data used by this thread
    
    iAction = 0;
    iAction = pDev->pStreamOn->Wait(0) ? 1 : 0;
    if(iAction == 0) iAction = pDev->pStreamOff->Wait(0) ? 2 : 0;
    switch(iAction)
    {
      case 1: //stream on
        bDeadSocketReported = false;
        bFirstFrame = true;
        pDev->bStreamRunning = 1;
        break;
      case 2: //stream off
        pDev->bStreamRunning = 0;
        break;
      case 0: //no event
        bool bRunning = pDev->bStreamRunning;
        {
          //ethernet
          C_Socket* pImageSocket = pDev->pTCPImage;
          pImageSocket->WaitForWriteEvent(0); //detects socket close
          if(!pImageSocket->GetSocketActive())
          {
            //dead socket, must reconnect
            //cannot set pDev->bStreamRunning to 0 here, it can hang StreamOn
            if(!bDeadSocketReported) gpLog->Print(LOG_WARN, "[%4x] StreamTread (eth): Image socket is dead, need to reconnect!\n", hDev);
            bDeadSocketReported = true;
            iSleep = 80;
            goto skiptonext;
          }
          
          if(!bRunning)
          {
            //stop TCP transfers if running (recv data to NULL)
            pImageSocket->WaitForReadEvent(0);
            int f = pImageSocket->FlushRecvBuffer();
            if(f) gpLog->Print(LOG_INFO, "[%4x] StreamTread (eth): Flushed data (%d bytes)\n", hDev, f);
          }
          else
          {
            int iLen;
            FZ_LL_EthImageHeader stEthHeader;
            //resync:
            //get header
            iLen = sizeof(stEthHeader);
            if(!pImageSocket->GetRecvBuffer((char*)&stEthHeader, &iLen, 1500))
            {
              gpLog->Print(LOG_ERROR, "[%4x] StreamTread (eth): Error in header retrieval!\n", hDev);
              iSleep = 1;
              goto skiptonext;
            }
            //validate header
            if(strncmp(stEthHeader.szMagic, "FZB3", 4) != 0 || ntohl(stEthHeader.iSize) != sizeof(stEthHeader) || ntohl(stEthHeader.iType) != 3)
            {
              //out of sync, we should either disconnect here (demands reconnect),
              // or scan for header to resync, because we will probably never get in sync again otherwise
              gpLog->Print(LOG_WARN, "[%4x] StreamTread (eth): Error in header, resync!\n", hDev);
              
              //get new data and scan for header marker and set current stream position to the found position
              //this resync procedure is needed because:
              //-the stream off action is done just before sending the stop command, but even if we change it to after
              //  the stop there are no guarantees that we always remain in sync (even if we remove the FlushRecvBuffer above).
              //-the stream off action happens in sync with this thread but the stop command is sent on another socket,
              //  so there are no guarantees that we wont receive extra images after a stop is sent, since the camera may
              //  not stop sending images before sending reply to the stop command (which is wrong).
              bool bFoundHeaderStart = false;
              unsigned char* pf = pDev->buf; //current frame, used here as a temp buffer
#define SCANSIZE (1024*2)
#define MAX_COUNT (((640*480*8)/SCANSIZE)+2) //set slightly more than one frame
              iLen = SCANSIZE; //read per loop
              //TODO: we miss if the header is within the bytes we have already received to stEthHeader, but it just skips an extra frame if this happens
              int iCounter = 0;
              bool bError = !pImageSocket->PeekRecvBuffer((char*)pf, &iLen, 2000); //this may pass a frame boundry, and with low framerate it takes longer
              while(!bFoundHeaderStart && !bError && iCounter < MAX_COUNT)
              {
                int iGuessedPos = ScanForEthImgHeader((char*)pf, iLen);
                if(iGuessedPos >= 0)
                {
                  bFoundHeaderStart = true;
                  iLen = iGuessedPos;
                }
                else
                {
                  iLen -= 4; //save the last 4 to the next loop
                }
                pImageSocket->GetRecvBuffer((char*)pf, &iLen, 0); //skip forward in data we already have
                iLen = SCANSIZE;
                if(!bFoundHeaderStart)
                  bError = !pImageSocket->PeekRecvBuffer((char*)pf, &iLen, 2000); //this may pass a frame boundry, and with low framerate it takes longer
                iCounter++;
              }
              if(bFoundHeaderStart)
                gpLog->Print(LOG_INFO, "[%4x] StreamTread (eth): Resync, possible header start found\n", hDev);
              else
              {
                if(bError) gpLog->Print(LOG_WARN,  "[%4x] StreamTread (eth): Error in header, could not resync (no new data received)!\n", hDev);
                else       gpLog->Print(LOG_ERROR, "[%4x] StreamTread (eth): Error in header, could not resync (header not found in last %d bytes)!\n", hDev, MAX_COUNT * SCANSIZE);
                iSleep = 1;
              }
              
              goto skiptonext; //next main loop will try the new position
            }
            
            //get image
            unsigned char* pf = pDev->buf; //current frame
            int ncols = stEthHeader.stHeader.ncols;
            int nrows = stEthHeader.stHeader.nrows;
            int bpp = stEthHeader.stHeader.bytesperpixel;
            int iRowHeaderLen = stEthHeader.stHeader.version >= 1 ? 0 : 8 * nrows;
            iLen = bpp * ncols * nrows + iRowHeaderLen;
            if(iLen > (FZ_MAX_ROWS_PER_FRAME * MAX_BYTES_PER_LINE))
            {
              gpLog->Print(LOG_ERROR, "[%4x] StreamTread (eth): Bad header! Too large image (bpp %d, w %d, h %d)\n", hDev, bpp, ncols, nrows);
              iSleep = 1;
              goto skiptonext;
            }
            if(!pImageSocket->GetRecvBuffer((char*)pf, &iLen, 1500))
            {
              gpLog->Print(LOG_ERROR, "[%4x] StreamTread (eth): Error in image retrieval! (asked=%d got=%d)\n", hDev, bpp * ncols * nrows + (nrows * 8), iLen);
              iSleep = 1;
              goto skiptonext;
            }
            //when we get here we have received data for a full frame
            
            if(!bFirstFrame && (iLastFrame + 1 != stEthHeader.stHeader.processedframecounter))
            {
              gpLog->Print(LOG_TRACE, "[%4x] StreamTread (eth): Frame skipped: last frame %d (this frame %d)\n", hDev, iLastFrame, stEthHeader.stHeader.processedframecounter);
              //changed from LOG_WARN to LOG_TRACE because of the frame skip command now implemented
            }
            if(!bFirstFrame && (stEthHeader.stHeader.lasterrorframe == stEthHeader.stHeader.processedframecounter))
            {
              gpLog->Print(LOG_WARN, "[%4x] StreamTread (eth): Camera frame error (this frame %d)\n", hDev, stEthHeader.stHeader.processedframecounter);
            }
            iLastFrame = stEthHeader.stHeader.processedframecounter;
            bFirstFrame = false;
            
            //store the frame rate count in the header
            pDev->iCurrentFrame = stEthHeader.stHeader.processedframecounter;
            stEthHeader.stHeader.measuredframerate = pDev->iMeasuredFramesPerSecond;
            
            //the current buffer to place the frame in
            iCurrBufNr = pDev->currdrvbuf;
            
            //copy the frame to the framequeue
            //we are able to always copy the data to a buffer even if the queue is found to be full in push,
            // since we have one extra buffer compared to the max queue size
            
            //header
            memcpy(&pDev->dheadq[iCurrBufNr], &stEthHeader.stHeader, sizeof(stEthHeader.stHeader));
            
            //stEthHeader.stHeader.version 0 has row headers
            //stEthHeader.stHeader.version 1 has no row headers
            
            //image rows, always returned as width*8 bytes (even for raw mode which is only width*4 bytes valid data)
            if(stEthHeader.stHeader.version >= 1)
            {
              memcpy(pDev->dbufq[iCurrBufNr], pf, nrows * ncols * bpp);
            }
            else
            {
              for(int i = 0; i < nrows; i++)
              {
                memcpy(pDev->dbufq[iCurrBufNr] + (i * (ncols * bpp)), pf + 8, ncols * bpp);
                pf += ncols * bpp + 8; //ncols*8 for XYZB data + 8 lineheader
              }
            }
          }
        }
        if(bRunning)
        {
          //when we get here we have received data for a full frame,
          // and we have copied the frame to iCurrBufNr
          
          //put the buffer in the frame queue
          //if queue overflow, the oldest frame is wasted and this new becomes last
          if(StreamQueuePush(pDev, iCurrBufNr) > -1)
          {
            pDev->currdrvbuf = (iCurrBufNr + 1) % (QUEUE_BUFFERS + 1); //only inc currdrvbuf if not overflow (ie was pushed)
            //NOTE: StreamQueuePush() now fixed so this will always happen
          }
          
          //signal that a frame is ready
          pDev->pFrameReady->Signal();
          
          //calculate frame rate
          pDev->iFcount++;
          time(&tv);
          if(tv != pDev->tSec)
          {
            pDev->tSec = tv;
            pDev->iMeasuredFramesPerSecond = pDev->iFcount;
#ifdef _DEBUG
            printf("[%4x] StreamThread: %d Frames/sec\r", (int)hDev, pDev->iFcount);
#endif
            pDev->iFcount = 0;
          }
        }
        else
        {
          //sleep to avoid taking CPU
          iSleep = 10;
        }
    }
    
skiptonext:
    if(iSleep > 0) mssleep(iSleep); //sleep last
  }
  
  pDev->bStreamRunning = false; //should be false already
  delete pDev->pStreamOn;
  pDev->pStreamOn = NULL;
  delete pDev->pStreamOff;
  pDev->pStreamOff = NULL;
  delete pDev->pQueueMutex;
  pDev->pQueueMutex = NULL;
  delete pDev->pFrameReady;
  pDev->pFrameReady = NULL;
  gpLog->Print(LOG_INFO, "[%4x] StreamThread: Exiting\n", hDev);
  pDev->bRemovalPending = false;
  return THREAD_RET_0;
}
