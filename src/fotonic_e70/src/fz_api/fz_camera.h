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

#ifndef __FZ_CAMERA_HEADER__
#define __FZ_CAMERA_HEADER__

#include "fz_api.h"

//the contents of this file is shared between the Camera and PC compilers

//frame info
#define MAX_BYTES_PER_LINE   (FZ_MAX_COLS_PER_FRAME*FZ_MAX_BYTES_PER_PIXEL+8)

//other
#define FZ_LL_PARAMETER_MAX_SIZE  (420)

#pragma pack(4)
typedef struct
{
  uint32_t serialn;
  uint16_t code;
  uint16_t bytelen;
  uint16_t offset;
  uint32_t iReturnBytesRequested;
  union
  {
    uint8_t  prm[FZ_LL_PARAMETER_MAX_SIZE];
    int16_t  prm16[FZ_LL_PARAMETER_MAX_SIZE / 2];
  };
} FZ_LL_Pkt_t;
#pragma pack()

////////////////////////////////////////////////////////////////////////////////
//for ethernet support

#define FZ_BROADCASTPORT 1288   //port on camera (known to pc and camera)

#pragma pack(2)
typedef struct
{
  //stored in network order
  uint32_t iSize;       //sizeof(struct)
  char szMagic[4];      //"FZB3"
  uint32_t iType;       //must be 0
  //uint32_t iReplyIP; <- must be fetched by camera from packet (since pc may send this on many interfaces)
  uint32_t iReplyPort;  //port that camera shall send response to
  uint32_t iCommand;    //extra command sent in broadcast (0 for normal operation)
} FZ_LL_EthPCtoFZBroadCast;

typedef struct
{
  //stored in network order
  uint32_t iSize;       //sizeof(struct)
  char szMagic[4];      //"FZB3"
  uint32_t iType;       //must be 1 or 10 (jaguar or panasonic)
  uint32_t iDeviceIP;            //ip of camera
  uint32_t iDeviceListeningPort; //port on camera that pc shall connect TCP socket(s) to
  // shall be FZ_TCPPORT
  char szSerial[16];             //NULL terminated serial number string, "N/A" if not programmed in camera
} FZ_LL_EthFZtoPCUDPReply;

typedef struct
{
  //stored in network order
  uint32_t iSize;       //sizeof(struct)
  char szMagic[4];      //"FZB3"
  uint32_t iType;       //must be 2
  //stored in camera order (same as PC)
  FZ_LL_Pkt_t stCmd;
} FZ_LL_EthCommand;

typedef struct
{
  //stored in network order
  uint32_t iSize;       //sizeof(struct)
  char szMagic[4];      //"FZB3"
  uint32_t iType;       //must be 3
  //stored in camera order (same as PC)
  FZ_FRAME_HEADER_EXT stHeader;
} FZ_LL_EthImageHeader;
#pragma pack()

#endif
