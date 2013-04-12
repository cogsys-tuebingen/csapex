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
#include "fz_ll.h"
#include "logger.h"

//globals
static C_Mutex* gpMutex = NULL;
volatile bool gbIsInitialized = false;         //used from calib_load.cpp, fzapi_channel.cpp, ll files
C_Logger* gpLog = NULL;                        //used from calib_load.cpp, fzapi_channel.cpp, ll files
FZ_Device_Context_t gDevices[MAX_API_DEVICES]; //used from calib_load.cpp
C_Mutex* gpMutex2 = NULL;                      //used from fzapi_channel.cpp
FZ_Channel gChannels[MAX_CHANNELS];            //used from fzapi_channel.cpp

#define FZAPI_VERSION "v4.1.0" // VS_VERSION_INFO should be changed for windows

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//driver init/deinit

bool DriverInit()
{
  if(gbIsInitialized) return true;
  
  //the mutex is needed to make sure init is done only once
  // if many threads are calling the DLL before it is initialized
  gpMutex->Enter(); //lock
  if(gbIsInitialized)
  {
    gpMutex->Leave(); //unlock
    return true;
  }
  
  gpLog = new C_Logger();
#if _DEBUG
  gpLog->SetLogLevel(LOG_ALL & ~LOG_TRACE);
  gpLog->SetOutLevel(LOG_TO_STDOUT | LOG_OPEN_CONSOLE);
#endif
  
  memset(gChannels, 0, sizeof(gChannels));
  memset(gDevices, 0, sizeof(gDevices));
  FZ_LL_Init();
  gbIsInitialized = true;
  gpMutex->Leave(); //unlock
  gpLog->Print(LOG_INFO, "[____] FZ_API DriverInit: Done\n");
  
  return true;
}

bool DriverCleanup()
{
  if(gpLog)
  {
    delete gpLog;
    gpLog = NULL;
  }
  if(gpMutex)
  {
    delete gpMutex;
    gpMutex = NULL;
  }
  if(gpMutex2)
  {
    delete gpMutex2;
    gpMutex2 = NULL;
  }
  
  gbIsInitialized = false;
  return true;
}

FZ_API FZ_Result FZ_Init()
{
  if(!gpMutex) gpMutex = new C_Mutex(); //in windows this is done in DLL_PROCESS_ATTACH
  if(!gpMutex2) gpMutex2 = new C_Mutex(); //in windows this is done in DLL_PROCESS_ATTACH
  FZ_Result tResult = DriverInit() ? FZ_Success : FZ_Failure;
  FZ_LL_Init();
  return tResult;
}

FZ_API FZ_Result FZ_Exit()
{
  if(!gbIsInitialized)
  {
    return FZ_NOT_INITIALIZED;
  }
  
  FZ_LL_Exit();
  return DriverCleanup() ? FZ_Success : FZ_Failure;
}

#ifdef WIN32
#ifdef FZAPI_EXPORTS
BOOL APIENTRY DllMain(
  HMODULE hModule,
  DWORD  iReason,
  LPVOID lpReserved)
{
  switch(iReason)
  {
    case DLL_PROCESS_ATTACH:
      //this is only called once per process,
      // and the dll data is local to that process, no need
      // use reference-counter, it is handled by Windows.
      
      //do as little as possible here due to all the problems
      // with what is forbidden to do from DllMain.
      //gbIsInitialized is checked from all exported functions
      // and perform the initialization if needed (first time)
      //need to create mutex here for use in DriverInit()
      gpMutex = new C_Mutex();
      gpMutex2 = new C_Mutex();
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
      break;
    case DLL_PROCESS_DETACH:
      //same as for DLL_PROCESS_ATTACH
      if(gbIsInitialized)
      {
        printf("FZ_API DLL_PROCESS_DETACH: still initialized (FZ_Exit() not run)");
      }
      //new: FZ_Exit() must be run when the driver is not beeing used any more.
      //DriverCleanup();
      break;
  }
  return TRUE;
}
#endif
#endif

FZ_API FZ_Result FZ_SetLogging(
  int iFlags,
  const char* szFilename,
  FZ_LOGCALLBACKTYPE pFunction)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  
  if((iFlags & FZ_LOG_TO_FILE) && !szFilename) return FZ_BAD_PARAMETERS;
  
  gpLog->SetLogLevel(iFlags);
  gpLog->SetOutLevel(iFlags);
  gpLog->AssignFile(szFilename);
  gpLog->AssignCallback(pFunction);
  
  return FZ_Success;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

int GetDeviceIndex(FZ_Device_Handle_t hDev)
{
  int i;
  bool bFound = false;
  
  for(i = 0; i < MAX_API_DEVICES; i++)
  {
    if(gDevices[i].opened && gDevices[i].llhandle == hDev)
    {
      bFound = true;
      break;
    }
  }
  
  if(!bFound) return -1;
  return i;
}

FZ_Result FZ_ReadVersionString(void* pOut, int* iLength)
{
  if(!pOut || !iLength || *iLength <= 0) return FZ_BAD_PARAMETERS;
  char szTmp[2048];
  char* szOut = (char*)pOut;
  
  strcpy(szTmp, "fz_api "FZAPI_VERSION);
  
  char szVersion[128];
  sprintf(szVersion, " (%s %s)", sizeof(void*) == 8 ? "x64" : "x86", __DATE__);
  strcat(szTmp, szVersion);
  
  strncpy(szOut, szTmp, *iLength);
  szOut[*iLength - 1] = 0;
  
  *iLength = (int)strlen(szOut) + 1; //length + null (all modified bytes)
  return FZ_Success;
}

FZ_API FZ_Result FZ_EnumDevices2(
  FZ_DEVICE_INFO* pDeviceInfo,
  int* piNumDevices)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  
  return FZ_LL_EnumDevices(pDeviceInfo, piNumDevices);
}

FZ_API FZ_Result FZ_EnumDevices(
  char** pszDevicePaths,
  int iMaxDevicePathLen,
  int* piNumDevices)
{
  //function depricated, supported through FZ_EnumDevices2
  if(!piNumDevices || !pszDevicePaths || *piNumDevices <= 0 || iMaxDevicePathLen < 24 || *piNumDevices > 256)
    return FZ_BAD_PARAMETERS;
    
  FZ_DEVICE_INFO* pDevices = new FZ_DEVICE_INFO[*piNumDevices];
  FZ_Result iResult = FZ_EnumDevices2(pDevices, piNumDevices);
  int i;
  for(i = 0; i < *piNumDevices; i++)
  {
    strncpy(pszDevicePaths[i], pDevices[i].szPath, iMaxDevicePathLen);
    pszDevicePaths[i][iMaxDevicePathLen - 1] = 0;
  }
  delete[] pDevices;
  
  return iResult;
}

FZ_API FZ_Result FZ_GetSimpleDeviceName(
  char* szDevicePath,
  char* szShortName,
  int iShortNameLen)
{
  //function depricated, still supported
  return FZ_LL_GetSimpleDeviceName(szDevicePath, szShortName, iShortNameLen);
}

int GetFreeDeviceIndex(FZ_Device_Handle_t hDev)
{
  int i;
  bool bFound = false;
  
  gpMutex->Enter(); //lock
  
  for(i = 0; i < MAX_API_DEVICES; i++)
  {
    if(gDevices[i].opened == 0)
    {
      gDevices[i].opened = 1;
      gDevices[i].llhandle = hDev;
      bFound = true;
      break;
    }
  }
  
  gpMutex->Leave(); //unlock
  
  if(!bFound) return -1;
  return i;
}

FZ_API FZ_Result FZ_Open(
  const char* szDevicePath,
  unsigned int iFlags,
  FZ_Device_Handle_t* phDev)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  
  FZ_Result iResult = FZ_Failure;
  bool demand_calib = (iFlags & FZ_FLAG_NO_CFG_CALIB) ? false : true;
  FZ_Device_Handle_t hDev;
  FZ_CmdRespCode_t rspcode;
  int iReturnedBytes, iReturnSize;
  int iWoiResult = FZ_Success;
  short has_calib_config = 0;
  
  gpLog->Print(LOG_TRACE, "[____] FZ_API Open: Begin device open (%s)\n", szDevicePath);
  
  if(!phDev || !szDevicePath) return FZ_BAD_PARAMETERS;
  
  //open low level handle
  iResult = FZ_LL_Open(szDevicePath, &hDev);
  if(iResult != FZ_Success)
  {
    int iLogLevel = LOG_ERROR;
    if(iResult == FZ_DEVICE_BUSY) iLogLevel = LOG_WARN; //exclude busy as an error since it is normal
    gpLog->Print(iLogLevel, "[____] FZ_API Open: Failed to open (%s) (code 0x%04x)\n", szDevicePath, iResult);
    return iResult;
  }
  iResult = FZ_Failure; //assume failure
  
  int iDev = GetFreeDeviceIndex(hDev);
  if(iDev < 0)
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_API Open: No free device index\n", hDev);
    goto open_exit;
  }
  else
  {
    gpLog->Print(LOG_TRACE, "[%4x] FZ_API Open: Device is assigned index %d\n", hDev, iDev);
  }
  *phDev = hDev;
  memset(&gDevices[iDev], 0, sizeof(FZ_Device_Context_t));
  gDevices[iDev].shutter1 = gDevices[iDev].shutter2 = 10;
  gDevices[iDev].woi_x = gDevices[iDev].woi_y = -1;
  gDevices[iDev].woi_w = gDevices[iDev].woi_h = -1;
  gDevices[iDev].fmt_flags = FZ_FMT_COMPONENT_B | FZ_FMT_COMPONENT_Z | FZ_FMT_COMPONENT_XY | FZ_FMT_PIXEL_PER_PLANE;
  
  gDevices[iDev].llhandle = hDev;
  gDevices[iDev].opened = 1;
  strncpy(gDevices[iDev].devicedescription, szDevicePath, sizeof(gDevices[iDev].devicedescription));
  gDevices[iDev].devicedescription[sizeof(gDevices[iDev].devicedescription) - 1] = 0;
  
  //stop sensor in case it was running
  FZ_IOCtl(hDev,
           CMD_DE_SENSOR_STOP,
           NULL, 0,
           &rspcode, NULL, NULL);
           
  //get sw version for communication core
  iReturnSize = iReturnedBytes = sizeof(gDevices[iDev].ca_version);
  if(FZ_Success != FZ_IOCtl(hDev,
                            CMD_CA_GET_VERSION,
                            NULL, 0,
                            &rspcode, &(gDevices[iDev].ca_version), &iReturnedBytes))
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_API Open: Unable to get CA version\n", hDev);
    goto open_exit;
  }
  gDevices[iDev].ca_version[iReturnSize - 1] = 0; //null terminate if not done (should be done in camera)
  gpLog->Print(LOG_INFO, "[%4x] FZ_API Open: CA version [%s]\n", hDev, gDevices[iDev].ca_version);
  
  //get sw version for depth engine core
  iReturnSize = iReturnedBytes = sizeof(gDevices[iDev].de_version);
  if(FZ_Success != FZ_IOCtl(hDev,
                            CMD_DE_GET_VERSION,
                            NULL, 0,
                            &rspcode, &(gDevices[iDev].de_version), &iReturnedBytes))
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_API Open: Unable to get DE version\n", hDev);
    goto open_exit;
  }
  gDevices[iDev].de_version[iReturnSize - 1] = 0; //null terminate if not done (should be done in camera)
  gpLog->Print(LOG_INFO, "[%4x] FZ_API Open: DE version [%s]\n", hDev, gDevices[iDev].de_version);
  
  //get product code
  iReturnSize = iReturnedBytes = sizeof(gDevices[iDev].productcode);
  if(FZ_Success != FZ_IOCtl(hDev,
                            CMD_DE_GET_PCODE,
                            NULL, 0,
                            &rspcode, &(gDevices[iDev].productcode), &iReturnedBytes))
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_API Open: Unable to get product code\n", hDev);
    goto open_exit;
  }
  gDevices[iDev].productcode[iReturnSize - 1] = 0; //null terminate if not done (should be done in camera)
  gpLog->Print(LOG_INFO, "[%4x] FZ_API Open: Product Code [%s]\n", hDev, gDevices[iDev].productcode);
  
  //get serial number
  iReturnSize = iReturnedBytes = sizeof(gDevices[iDev].unitno);
  if(FZ_Success != FZ_IOCtl(hDev,
                            CMD_DE_GET_UNIT_NO,
                            NULL, 0,
                            &rspcode, &(gDevices[iDev].unitno), &iReturnedBytes))
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_API Open: Unable to get unit number\n", hDev);
    goto open_exit;
  }
  gDevices[iDev].unitno[iReturnSize - 1] = 0; //null terminate if not done (should be done in camera)
  gpLog->Print(LOG_INFO, "[%4x] FZ_API Open: Unit Number [%s]\n", hDev, gDevices[iDev].unitno);
  
  //get device type (working only on panasonic, type 1)
  gDevices[iDev].devicetype = FZ_DEVICE_TYPE_JAGUAR; //default (jaguar fails CMD_DE_GET_DEVICE_TYPE)
  iReturnSize = iReturnedBytes = sizeof(short);
  if(FZ_Success != FZ_IOCtl(hDev,
                            CMD_DE_GET_DEVICE_TYPE,
                            NULL, 0,
                            &rspcode, &(gDevices[iDev].devicetype), &iReturnedBytes))
  {
  }
  gpLog->Print(LOG_INFO, "[%4x] FZ_API Open: Device Type %d\n", hDev, gDevices[iDev].devicetype);
  
  //get some more capabilities
  
  //the following or operator code works because FZ_Success is 0
  // ideally the 4 following commands should be only one...
  iReturnedBytes = sizeof(short);
  iWoiResult |= (int)FZ_IOCtl(hDev,
                              CMD_DE_GET_WOI_TOP,
                              NULL, 0, &rspcode, &(gDevices[iDev].woi_top), &iReturnedBytes);
  iReturnedBytes = sizeof(short);
  iWoiResult |= (int)FZ_IOCtl(hDev,
                              CMD_DE_GET_WOI_BOTTOM,
                              NULL, 0, &rspcode, &(gDevices[iDev].woi_bottom), &iReturnedBytes);
  iReturnedBytes = sizeof(short);
  iWoiResult |= (int)FZ_IOCtl(hDev,
                              CMD_DE_GET_WOI_LEFT,
                              NULL, 0, &rspcode, &(gDevices[iDev].woi_left), &iReturnedBytes);
  iReturnedBytes = sizeof(short);
  iWoiResult |= (int)FZ_IOCtl(hDev,
                              CMD_DE_GET_WOI_RIGHT,
                              NULL, 0, &rspcode, &(gDevices[iDev].woi_right), &iReturnedBytes);
  if(iWoiResult != FZ_Success)
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_API Open: Unable to get WOI\n", hDev);
    goto open_exit;
  }
  gDevices[iDev].ncols = (gDevices[iDev].woi_right - gDevices[iDev].woi_left) + 1;
  gDevices[iDev].nrows = (gDevices[iDev].woi_bottom - gDevices[iDev].woi_top) + 1;
  gpLog->Print(LOG_INFO, "[%4x] FZ_API Open: WOI is %dx%d\n", hDev, gDevices[iDev].ncols, gDevices[iDev].nrows);
  
  //get calib and config status
  iReturnedBytes = sizeof(short);
  if(FZ_Success != FZ_IOCtl(hDev,
                            CMD_DE_GET_FLASH_CONFIG,
                            NULL, 0, &rspcode, &has_calib_config, &iReturnedBytes))
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_API Open: Unable to get configuration status\n", hDev);
    goto open_exit;
  }
  
  if(demand_calib && has_calib_config == false)
  {
    //the camera does NOT have valid calib/config info
    gpLog->Print(LOG_ERROR, "[%4x] FZ_API Open: Device has no calib/config\n", hDev);
    iResult = FZ_INCORRECT_CALIBCONFIG;
    goto open_exit;
  }
  
  gpLog->Print(LOG_INFO, "[%4x] FZ_API Open: Device Opened\n", hDev);
  iResult = FZ_Success;
open_exit:
  if(iResult != FZ_Success)
  {
    FZ_Device_Handle_t llhandle = gDevices[iDev].llhandle;
    gDevices[iDev].opened = 0;
    FZ_LL_Close(llhandle);
  }
  
  return iResult;
}

FZ_API FZ_Result FZ_Close(FZ_Device_Handle_t hDev)
{
  if(!gbIsInitialized)
  {
    return FZ_NOT_INITIALIZED;
  }
  
  int iDev = GetDeviceIndex(hDev);
  if(iDev < 0) return FZ_BAD_HANDLE;
  
  //stop sensor
  FZ_CmdRespCode_t resp;
  FZ_IOCtl(hDev, CMD_DE_SENSOR_STOP, NULL, 0, &resp, NULL, NULL);
  
  FZ_Device_Handle_t llhandle = gDevices[iDev].llhandle;
  gDevices[iDev].opened = 0;
  FZ_LL_Close(llhandle);
  
  return FZ_Success;
}

FZ_API FZ_Result FZ_FrameAvailable(
  FZ_Device_Handle_t hDev)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  
  int iDev = GetDeviceIndex(hDev);
  if(iDev < 0) return FZ_BAD_HANDLE;
  
  return FZ_LL_FrameAvailable(hDev);
}

FZ_API FZ_Result FZ_SetFrameDataFmt(
  FZ_Device_Handle_t hDev,
  int x, int y,
  int w, int h,
  int iFlags)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  
  int iDev = GetDeviceIndex(hDev);
  if(iDev < 0) return FZ_BAD_HANDLE;
  
  //check flags
  if((iFlags & 0xff) == 0)
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_API FZ_SetFrameDataFmt: Trying to set data fmt to 0 components (flags %d)\n", hDev, iFlags);
    return FZ_BAD_PARAMETERS;
  }
  if(iFlags & FZ_FMT_COMPONENT_B && iFlags & FZ_FMT_COMPONENT_YUV422)
  {
    gpLog->Print(LOG_ERROR, "[%4x] FZ_API FZ_SetFrameDataFmt: Trying to set both B and YUV422 components (flags %d)\n", hDev, iFlags);
    return FZ_BAD_PARAMETERS;
  }
  if(gDevices[iDev].devicetype == FZ_DEVICE_TYPE_PRIMESENSE && iFlags & FZ_FMT_COMPONENT_B)
  {
    gpLog->Print(LOG_WARN, "[%4x] FZ_API FZ_SetFrameDataFmt: Trying to set B component on YUV422 camera (flags %d)\n", hDev, iFlags);
  }
  if(gDevices[iDev].devicetype != FZ_DEVICE_TYPE_PRIMESENSE && iFlags & FZ_FMT_COMPONENT_YUV422)
  {
    gpLog->Print(LOG_WARN, "[%4x] FZ_API FZ_SetFrameDataFmt: Trying to set YUV422 component on B camera (flags %d)\n", hDev, iFlags);
  }
  
  //check and set ROI
  if(w <= 0 || h <= 0)
  {
    //special case to set default (same as image)
    gDevices[iDev].woi_x = -1;
    gDevices[iDev].woi_y = -1;
    gDevices[iDev].woi_w = -1;
    gDevices[iDev].woi_h = -1;
  }
  else
  {
    if(x < -512 || y < -512 || w > FZ_MAX_COLS_PER_FRAME + 512 || h > FZ_MAX_ROWS_PER_FRAME + 512)
    {
      gpLog->Print(LOG_ERROR, "[%4x] FZ_API FZ_SetFrameDataFmt: Trying to set WOI with more than 512 pixels empty border (%d,%d [%dx%d])\n", hDev, x, y, w, h);
      return FZ_BAD_PARAMETERS;
    }
    if(x + w <= 0 || y + h <= 0 || x >= FZ_MAX_COLS_PER_FRAME || y >= FZ_MAX_ROWS_PER_FRAME)
    {
      gpLog->Print(LOG_ERROR, "[%4x] FZ_API FZ_SetFrameDataFmt: Trying to set WOI guaranteed to return no image data (%d,%d [%dx%d])\n", hDev, x, y, w, h);
      return FZ_BAD_PARAMETERS;
    }
    gDevices[iDev].woi_x = x;
    gDevices[iDev].woi_y = y;
    gDevices[iDev].woi_w = w;
    gDevices[iDev].woi_h = h;
  }
  
  //set flags (any combination of flags are allowed)
  gDevices[iDev].fmt_flags = iFlags;
  
  return FZ_Success;
}

FZ_API FZ_Result FZ_GetFrameExt(
  FZ_Device_Handle_t hDev,
  FZ_FRAME_HEADER_EXT* pHeader,
  void* pPixels, size_t* piPixelsByteLen)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  
  FZ_Result iResult = FZ_Success;
  FZ_LL_FRAME_HEADER stHeader;
  size_t iSize;
  
  if(!pHeader || !pPixels || !piPixelsByteLen)
  {
    return FZ_BAD_PARAMETERS;
  }
  
  int iDev = GetDeviceIndex(hDev);
  if(iDev < 0) return FZ_BAD_HANDLE;
  
  //check is now done in FZ_LL_GetFrame, since pixelsize is not always 8
  /*iSize = gDevices[iDev].nrows*gDevices[iDev].ncols*8;
  if(*piPixelsByteLen<iSize) {
    return FZ_BAD_PARAMETERS;
  }*/
  
  iSize = (int) * piPixelsByteLen;
  iResult = FZ_LL_GetFrame(gDevices[iDev].llhandle, &stHeader, pPixels, (int)iSize, gDevices[iDev].woi_x, gDevices[iDev].woi_y, gDevices[iDev].woi_w, gDevices[iDev].woi_h, gDevices[iDev].fmt_flags);
  
  if(iResult == FZ_Success)
  {
    //set returned size
    iSize = stHeader.nrows * stHeader.ncols * stHeader.bytesperpixel;
    *piPixelsByteLen = iSize;
    //set returned frame header info from low level header
    memcpy(pHeader, &stHeader, sizeof(FZ_FRAME_HEADER_EXT)); //must have the same fields and size
    if(gDevices[iDev].woi_w > 0) pHeader->ncols = (uint16_t)gDevices[iDev].woi_w;
    if(gDevices[iDev].woi_h > 0) pHeader->nrows = (uint16_t)gDevices[iDev].woi_h;
  }
  return iResult;
}

FZ_API FZ_Result FZ_GetFrame(
  FZ_Device_Handle_t hDev,
  FZ_FRAME_HEADER* pHeader,
  void* pPixels, size_t* piPixelsByteLen)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  
  FZ_FRAME_HEADER_EXT stHeader;
  
  if(!pHeader) return FZ_BAD_PARAMETERS;
  int iDev = GetDeviceIndex(hDev);
  if(iDev < 0) return FZ_BAD_HANDLE;
  
  FZ_Result iResult = FZ_GetFrameExt(hDev, &stHeader, pPixels, piPixelsByteLen);
  
  if(iResult == FZ_Success)
  {
    //set returned frame header info from extended (low level) header
    pHeader->bytesperpixel = stHeader.bytesperpixel;
    pHeader->reportedframerate = stHeader.reportedframerate;
    pHeader->measuredframerate = stHeader.measuredframerate;
    pHeader->lasterrorframe = stHeader.lasterrorframe;
    pHeader->mode = stHeader.mode;
    pHeader->ncols = stHeader.ncols;
    pHeader->nrows = stHeader.nrows;
    pHeader->framecounter = stHeader.processedframecounter;
    pHeader->shutter = stHeader.shutter;
    pHeader->temperature = stHeader.temperature;
    pHeader->version = (uint16_t)stHeader.version;
    pHeader->timestamp[0] = stHeader.ExpStartSec;  //start of exposure in seconds
    pHeader->timestamp[1] = stHeader.ExpStartMSec; //start of exposure milliseconds
    pHeader->timestamp[2] = stHeader.ExpDuration;  //delay from start of exposure to end of exposure
    pHeader->precision_b = 255;
    if(gDevices[iDev].devicetype == FZ_DEVICE_TYPE_PANASONIC) pHeader->precision_b = 1023;
    if(gDevices[iDev].devicetype == FZ_DEVICE_TYPE_JAGUAR) pHeader->precision_b = 2650;
    pHeader->format = gDevices[iDev].fmt_flags;
  }
  return iResult;
}

FZ_API FZ_Result FZ_GetFrameNewest(
  FZ_Device_Handle_t hDev,
  FZ_FRAME_HEADER* pHeader,
  void* pPixels, size_t* piPixelsByteLen)
{
  FZ_Result iRet = FZ_GetFrame(hDev, pHeader, pPixels, piPixelsByteLen);
  while(FZ_FrameAvailable(hDev) == FZ_Success)
  {
    iRet = FZ_GetFrame(hDev, pHeader, pPixels, piPixelsByteLen);
  }
  return iRet;
}

FZ_API FZ_Result FZ_IOCtl(
  FZ_Device_Handle_t hDev,
  FZ_CmdCode_t iCmd,
  void* pParam,
  int iCmdByteLen,
  FZ_CmdRespCode_t* piRespCode,
  void* pResp,
  int*  piRespByteLen)
{
  if(!gbIsInitialized)
  {
    if(!DriverInit()) return FZ_NOT_INITIALIZED;
  }
  
  FZ_Result iResult = FZ_Success;
  
  int iDev = GetDeviceIndex(hDev);
  if(iDev < 0) return FZ_BAD_HANDLE;
  
  if(gDevices[iDev].opened == 0)
  {
    return FZ_Failure;
  }
  
  if(iCmd == CMD_API_GET_VERSION)
  {
    *piRespCode = R_CMD_DE_ACK;
    iResult = FZ_ReadVersionString(pResp, piRespByteLen);
  }
  else
  {
    void* pParam2 = pParam;
    if(iCmd == CMD_DE_SET_MODE)
    {
      gDevices[iDev].mode = *(short*)pParam;
    }
    
    //hacks for new shutter cmd fmt on FZ-E70+
    if(gDevices[iDev].productcode[3] < 'E')
    {
      if(iCmd == CMD_DE_SET_SHUTTER)
      {
        //save shutter 1
        gDevices[iDev].shutter1 = *(short*)pParam;
      }
      else if(iCmd == CMD_DE_SET_SHUTTER_EXT)
      {
        short shutter1 = gDevices[iDev].shutter1;
        short shutter2 = gDevices[iDev].shutter2;
        FZ_SHUTTER_EXT* shutters = (FZ_SHUTTER_EXT*)pParam;
        //get new shutters
        for(int i = 0; i < shutters->num_shutters; i++)
        {
          if(shutters->shutters[i].shutter_nr == 1) shutter1 = shutters->shutters[i].shutter_time;
          if(shutters->shutters[i].shutter_nr == 2) shutter2 = shutters->shutters[i].shutter_time;
        }
        ((short*)pParam)[0] = 2;
        ((short*)pParam)[1] = shutter1;
        ((short*)pParam)[2] = shutter2;
        //save shutters 1 and 2
        gDevices[iDev].shutter1 = shutter1;
        gDevices[iDev].shutter2 = shutter2;
      }
    }
    
    iResult = FZ_LL_SendCommand(gDevices[iDev].llhandle,
                                iCmd,
                                (unsigned char*)pParam2, //pointer to parameter
                                iCmdByteLen,             //paramter size (bytes)
                                piRespCode,              //pointer to response code
                                (unsigned char*)pResp,   //pointer to response parameter
                                piRespByteLen,
                                0,
                                piRespByteLen ? *piRespByteLen : 0);
  }
  
  //error:
  return iResult;
}
