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

#ifndef __FZ_LL_HEADER__
#define __FZ_LL_HEADER__

#include "fz_internal.h"

//FZ_Result is the same error type as used in fz_api.h
// this is to have the same type from the lowest level
// and up to the user.

FZ_Result FZ_LL_Init();
FZ_Result FZ_LL_Exit();

FZ_Result FZ_LL_GetSimpleDeviceName(
  char* szDevicePath,
  char* szShortName,
  int iShortNameLen);

FZ_Result FZ_LL_EnumDevices(
  FZ_DEVICE_INFO* pDeviceInfo,
  int* piNumDevices);

FZ_Result FZ_LL_Open(
  const char* szDevicePath,
  FZ_Device_Handle_t* phDev);

FZ_Result FZ_LL_Close(
  FZ_Device_Handle_t hDev);

FZ_Result FZ_LL_SendCommand(
  FZ_Device_Handle_t hDev,    //device handle
  unsigned short iCmd,        //command code
  unsigned char* pParam,      //pointer to parameter
  int iParamSize,             //paramter size (bytes)
  unsigned short* piRespCode, //pointer to response code
  unsigned char* pRespParam,  //pointer to response parameter
  int* piRespSize,            //pointer to response parameter size
  unsigned short iOffset,
  unsigned int iReturnBytesRequested);

FZ_Result FZ_LL_FrameAvailable(
  FZ_Device_Handle_t hDev);

FZ_Result FZ_LL_GetFrame(
  FZ_Device_Handle_t hDev,
  void* pHeader,
  void* pPixels,
  int iPixelsByteLen,
  int x, int y,
  int w, int h,
  int iFlags);

#endif
