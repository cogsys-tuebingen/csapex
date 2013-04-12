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

#ifndef __FZ_LL_INTERNAL_HEADER__
#define __FZ_LL_INTERNAL_HEADER__

#include "socket_2.h" //must be included before windows.h
#include "common.h"

#include "fz_internal.h"
#include "fz_ll.h"

//internal return codes
#define FZ_LL_SUCCESS   0x00
#define FZ_LL_FAIL      0x01

#define QUEUE_BUFFERS 5

#define FZ_DEVICE_TYPE_JAGUAR_CA     1
#define FZ_DEVICE_TYPE_PANASONIC_CA  10
#define FZ_DEVICE_TYPE_PRIMESENSE_CA 20

typedef struct FZ_Device_LL
{
  struct FZ_Device_LL* pNext;
  
  //device variables
  char szDevicePath[2048];
  FZ_Device_Handle_t iDeviceNum;
  int iCurrentCmdTimeout;
  
  C_Mutex* pCommandMutex;
  
  //ethernet device variables
  C_Socket* pTCPCommand;
  C_Socket* pTCPImage;
  
  //stats
  time_t tSec;
  unsigned int iFcount;
  unsigned int iCurrentFrame;
  int iMeasuredFramesPerSecond;
  
  //stream control
  volatile bool bStreamRunning;
  volatile bool bRemovalPending;
  C_Event* pStreamOn;
  C_Event* pStreamOff;
  C_Event* pFrameReady;
  C_Thread* pStreamThread;
  
  //buffers for images before they are retreived with GetImage()
  unsigned char buf[(FZ_MAX_ROWS_PER_FRAME + 1) * (MAX_BYTES_PER_LINE)];            //receive buffer
  unsigned char dbufq[QUEUE_BUFFERS + 1][FZ_MAX_ROWS_PER_FRAME * (MAX_BYTES_PER_LINE)]; //enqueued images
  FZ_LL_FRAME_HEADER dheadq[QUEUE_BUFFERS + 1];                                    //header of enqueued images
  int currdrvbuf; //%(QUEUE_BUFFERS+1), need one extra to always have a buffer ready, even when the queue is full
  
  //push/pop queue impl
  volatile int aiQueueImgnr[QUEUE_BUFFERS]; //the same pos is never operated on by both push and pop
  volatile int iQueueSize; //shared, only atomic operations
  int iQueuePushPos; //only accessed by push
  int iQueuePopPos;  //only accessed by pop
  C_Mutex* pQueueMutex; //lock
} FZ_Device_LL;

typedef struct DRIVER_CONTEXT
{
  C_Mutex*      pMutex;
  FZ_Device_LL* pOpenDevices;  //open devices
} DRIVER_CONTEXT;

//declarations for functions in stream.cpp
int InitializeStreamThread(FZ_Device_Handle_t hDev);
int StreamOn(FZ_Device_Handle_t hDev);
int StreamOff(FZ_Device_Handle_t hDev);

FZ_Device_LL* GetOpenDeviceStruct(FZ_Device_Handle_t hDev);

#endif
