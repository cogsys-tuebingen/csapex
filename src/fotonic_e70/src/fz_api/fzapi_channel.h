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

#ifndef __FZ_CHANNEL_HEADER__
#define __FZ_CHANNEL_HEADER__

#include "socket_2.h" //must be included before windows.h

#define MAX_CHANNELS 20
typedef struct FZ_Channel
{
  volatile int iState; //0 closed, 1 listening, 2 connecting, 3 open to recv, 4 open to send
  C_Socket* pclSocket;
} FZ_Channel;

#endif
