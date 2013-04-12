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

//
// winsock2 network-class for TCP and UDP
// updated 2010 with broadcast support, and other improvements
// supports windows, linux and mac.
//

//this file must be included before windows.h

#ifndef _SOCKET2_H_
#define _SOCKET2_H_

#ifdef WIN32
//windows sockets
#include <winsock2.h> //must be included before windows.h
#define WSAEWOULDBLOCK2 WSAEWOULDBLOCK
#else
//linux sockets
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <net/if.h>
#include <arpa/inet.h>

#define WSAGetLastError() errno
#define WSAEWOULDBLOCK EINPROGRESS
#define WSAEWOULDBLOCK2 EAGAIN
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SOCKET int
#define ioctlsocket ioctl
#define closesocket close
#define SD_BOTH SHUT_RDWR
#endif

#include <vector>
#include <deque>

#if 0 //_DEBUG
#include "logger.h"
#define SocketLogText(text) \
   do { if(1) C_Logger::PrintToFile("net_log.txt", "%s\n", text); } while(0)
#else
#define SocketLogText(text)
#endif

#define SOCKET_BUFSIZE 4096  //buffer size

struct S_RecvData
{
  int iSrcIP, iSrcPort;
  int iLength;
  char pcBuf[SOCKET_BUFSIZE];
};

struct S_SendData
{
  int iDstIP, iDstPort; //only used for UDP
  int iLength;          //for UDP: dont send bigger than BUFSIZE for it to be handled in one buf by the other side (if this class is receiving)
  char* pData;
};

class C_Socket;
struct S_Event
{
  C_Socket* pSocket;
  int       iEvent;         //FD_CONNECT, FD_ACCEPT
  union
  {
    int      iResult;     //on FD_CONNECT
    C_Socket* pConnected; //on FD_ACCEPT
  };
};

struct S_InterfaceInfo
{
  char szIP[32];
  char szMask[32];
  char szBroadcast[32];
  char szGateway[32]; //not supported yet
  bool bActive;
  bool bBroadcast;
  bool bLoopBack;
};

class C_Socket
{
public:
  C_Socket();                  //to create a normal socket to connect to something later
  C_Socket(int i_iListenPort); //to create a listening socket
  ~C_Socket();
  
  int Disconnect();
  bool GetSocketActive()
  {
    return (m_hSock != INVALID_SOCKET);
  };
  
  //buffer methods
  int SendBufferSize()
  {
    return m_iBufferedSend;
  };
  void ClearSendQueue();
  int RecvBufferSize()
  {
    return m_iBufferedReceive;
  };
  bool PeekRecvBuffer(char* o_pcBuf, int* io_iLength, int i_iTimeout = 0, int* o_iIP = NULL, int* o_iPort = NULL); //returns true if buffer!=empty
  bool GetRecvBuffer(char* o_pcBuf, int* io_iLength, int i_iTimeout = 0, int* o_iIP = NULL, int* o_iPort = NULL);  //returns true if all received, false for error or timeout
  int FlushRecvBuffer(); //returns true if buffer!=empty
  
  //address methods (not valid before one object is created and WSAStartup has been run)
  static char* IPToString(int i_iIP, char* o_szResult, int i_iSize);
  static int ResolveAddress(const char* i_szHost);
  static bool GetLocalAddressInfo(S_InterfaceInfo* o_pstList, int* io_iMaxNum); //run to retrieve a list with extended interface info (using another method than the above).
  static bool IsLocalAddress(int i_iIP); //true if the input ip nr is the same as any of the local addresses
  
  bool Connect(const char* i_szAddr, int i_iPort, int iTimeout = 1000);
  bool Connect(int i_iAddr, int i_iPort, int iTimeout = 1000);
  bool SetNoDelay();
  
  int SendQueued(char* i_pData, int i_iLength); //sends data on a connected socket, always succeds if connected, data is copied and send later if socket buffer is full
  
  unsigned int ConnectedToIP(); //returns the destination ip, 0 if not connected (dead socket)
  int ListenOnPort()
  {
    return m_iListenOnPort;
  };
  
  //send/recv/accept/closed handling
  C_Socket* AcceptConnection(); //NULL if accept failed (no timeout)
  bool m_bAcceptWillSucceed;
  bool WaitForReadEvent(int iTimeout);
  bool WaitForWriteEvent(int iTimeout);
  
  //UDP functions
  int InitRecvUDP(int i_iPort = 0, bool i_bBroadcast = false); //returns port to recv on or -1
  bool InitSendUDP(int i_iAddr, int i_iPort, bool i_bBroadcast = false); //use SendQueued for connected/send inited UDP
  
private:
  bool SetNonBlocking();
  void UpdateQueued(); //sends queued data until socket buffer full (or queue empty)
  int Send(char* i_pData, int i_iLength); //sends data on a connected socket
  
  void Init();
  bool InitListen(int i_iPort);
  
  std::deque<S_SendData> m_clSendDataList;
  
  bool Receive();
  
  //socket variables
  SOCKET m_hSock; //the socket
  
  //socket info and statistics
  int  m_iBufferedReceive, m_iBufferedSend; //data currently buffered
  int  m_iTotByteRcvd, m_iTotByteSent;      //total bytes received and sent
  
  std::deque<S_RecvData*> *m_pRcvPacketList;
  
  sockaddr_in m_stAddrDest, m_stAddrLocal; //addresses and ports
  
  //state info
  bool m_bListening;
  int  m_iListenOnPort;
  int  m_iConnectResult; //only tcp yet
  bool m_bConnected;
  bool m_bUDP;
  
  //other
  void OutputErrorMsg(char* i_szText, int i_iErrorNr);
};

#endif
