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

#ifndef __FZ_INTERNAL_HEADER__
#define __FZ_INTERNAL_HEADER__

#include "fz_api.h"
#include "fz_camera.h"

#include "common.h"

#define MAX_API_DEVICES 40

typedef struct __FZ_Device_Context
{
  int opened; //indicates that the element is in use/valid
  
  //everything is set in FZ_Open
  FZ_Device_Handle_t llhandle;
  
  char devicedescription[1024];
  char de_version[128];
  char ca_version[128];
  char productcode[128];
  char unitno[11];
  
  //note: only used in device type 0 calibration
  short woi_top;
  short woi_bottom;
  short woi_left;
  short woi_right;
  short nrows;
  short ncols;
  
  //camera state
  short devicetype; //0, 1, 2 (jaguar, panasonic, primesense)
  short mode;
  short shutter1, shutter2;
  
  //api settings
  int fmt_flags;
  int woi_x, woi_y;
  int woi_w, woi_h;
} FZ_Device_Context_t;

#define FZ_LL_FRAME_HEADER FZ_FRAME_HEADER_EXT

//internal global functions
bool DriverInit();
int GetDeviceIndex(FZ_Device_Handle_t hDev);
FZ_Result LoadAndSendCalibFile(FZ_Device_Handle_t hDev, char* szFilename, char* szFilename2, char* szFilename3);
FZ_Result LoadAndSendLookupFile(FZ_Device_Handle_t hDev, char* szFilename, int iType);
FZ_Result SendConfigFile(FZ_Device_Handle_t hDev, int iType, void* pData, int iDataSize);
int LoadConfigFile(char* szFilename, unsigned char* pBuffer, int iLen);

#endif
