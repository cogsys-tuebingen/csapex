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

//define to log all sent/recved data to files
//#define DEBUG_SOCKET

#include "socket_2.h"
#ifdef WIN32
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#endif

#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////
//Init/Deinit

C_Socket::C_Socket(int i_iListenPort)
{
  Init();
  
  //create socket for listening
  InitListen(i_iListenPort);
}

C_Socket::C_Socket()
{
  Init();
}

void C_Socket::Init()
{
  m_hSock = INVALID_SOCKET;
  m_iConnectResult = -1;
  m_bConnected = false;
  m_bListening = false;
  m_iListenOnPort = -1;
  m_bUDP = false;
  m_bAcceptWillSucceed = false;
  
  memset(&m_stAddrLocal, 0, sizeof(m_stAddrLocal));
  memset(&m_stAddrDest,  0, sizeof(m_stAddrDest));
  
  //reset counters
  m_iTotByteSent = 0;
  m_iBufferedReceive = 0;
  m_iBufferedSend = 0;
  m_iTotByteRcvd = 0;
  
#ifdef WIN32
  //initiation of winsock
  WSADATA stWSAData;
  if(WSAStartup(0x0202, &stWSAData) != 0)
  {
    OutputErrorMsg((char*)"C_Socket::Init: WSAStartup: ", WSAGetLastError());
  }
#endif
  
  m_pRcvPacketList = new std::deque<S_RecvData*>;
}

C_Socket::~C_Socket()
{
  Disconnect();
  ClearSendQueue();
  
  size_t i, iSize = m_clSendDataList.size();
  for(i = 0; i < iSize; i++)
    delete &m_clSendDataList[i];
    
  iSize = m_pRcvPacketList->size();
  for(i = 0; i < iSize; i++)
    delete(*m_pRcvPacketList)[i];
  delete m_pRcvPacketList;
  m_pRcvPacketList = NULL;
  
#ifdef WIN32
  //close winsock
  if(WSACleanup() == SOCKET_ERROR)
  {
    OutputErrorMsg((char*)"C_Socket::~C_Socket: WSACleanup: ", WSAGetLastError());
  }
#endif
}

bool C_Socket::SetNonBlocking()
{
#ifdef WIN32
  unsigned long iNonBlock = 1;
  if(ioctlsocket(m_hSock, FIONBIO, &iNonBlock) == SOCKET_ERROR)
  {
    return false;
  }
#else
  int iFlags = fcntl(m_hSock, F_GETFL, 0);
  fcntl(m_hSock, F_SETFL, iFlags | O_NONBLOCK);
#endif
  return true;
}

bool C_Socket::SetNoDelay() //can be done on a connected TCP sockets
{
  unsigned long iNoDelay = 1;
  int iResult = setsockopt(m_hSock, IPPROTO_TCP, TCP_NODELAY,
                           (char*)&iNoDelay, sizeof(iNoDelay));
  return (iResult != SOCKET_ERROR);
}

////////////////////////////////////////////////////////////////////////////////
//TCP can be either listening or normal (send and recv)
//some also used for UDP

bool C_Socket::Connect(const char* i_szAddr, int i_iPort, int iTimeout)
{
  return Connect(ResolveAddress(i_szAddr), i_iPort, iTimeout);
}

bool C_Socket::Connect(int i_iAddr, int i_iPort, int iTimeout)
{
  int iRet;
  
  if(m_bListening) return false;
  if(m_hSock != INVALID_SOCKET) Disconnect(); //close socket if open
  
  ClearSendQueue();
  
  //get a new socket
  m_hSock = socket(AF_INET, SOCK_STREAM, 0);
  if(m_hSock < 0) //if error, report and return errorcode
  {
    SocketLogText("C_Socket::Connect: Error creating socket.");
    goto error;
  }
  m_bUDP = false;
  
  //set the local address
  //m_stAddrLocal.sin_family = AF_INET;         //IP
  //m_stAddrLocal.sin_port = htons(0);          //source port (any)
  //m_stAddrLocal.sin_addr.s_addr = INADDR_ANY; //any interface
  
  //set other address
  m_stAddrDest.sin_family = AF_INET;                      //IP
  m_stAddrDest.sin_port = htons((unsigned short)i_iPort); //init destination port
  m_stAddrDest.sin_addr.s_addr = i_iAddr;                //init remote Address
  
  //set socket non blocking
  if(!SetNonBlocking())
  {
    SocketLogText("C_Socket::Connect: Error setting socket non blocking.");
    goto error;
  }
  
  //connect TCP/IP sockets
  iRet = connect(m_hSock, (struct sockaddr*)&m_stAddrDest, sizeof(struct sockaddr_in));
  if(iRet == SOCKET_ERROR)
  {
    int iErr = WSAGetLastError();
    if(iErr != WSAEWOULDBLOCK)
    {
      OutputErrorMsg((char*)"C_Socket::Connect: ", iErr);
      goto error;
    }
  }
  
  if(iTimeout) WaitForWriteEvent(iTimeout); //block until connect or timeout
  //m_bConnected is set to true if connection succeeded within timeout
  
  return iTimeout ? m_bConnected : false;
error:
  closesocket(m_hSock);
  m_hSock = INVALID_SOCKET;
  return false;
}

int C_Socket::Disconnect()
{
  int iRet = 0;
  char pcDiscard[SOCKET_BUFSIZE];
  
  m_bConnected = false;
  m_iConnectResult = -1;
  if(m_hSock != INVALID_SOCKET)
  {
    //shutdown
    iRet = shutdown(m_hSock, SD_BOTH); //skip return val check, because it may fail
    
    //read remaining data (until EOF or error)
    for(iRet = 1; (iRet && (iRet != SOCKET_ERROR));)
    {
      //discard any data received
      iRet = (int)recv(m_hSock, pcDiscard, SOCKET_BUFSIZE, 0);
    }
    
    //close the socket, and ignore any error
    closesocket(m_hSock);
    m_hSock = INVALID_SOCKET;
  }
  return iRet;
}

bool C_Socket::InitListen(int i_iPort)
{
  int iRet;
  sockaddr_in stAddr;
  socklen_t iLen = sizeof(stAddr);
  unsigned long iReuseAddr = 1;
  
  //socket must not be connected
  if(m_hSock != INVALID_SOCKET)
  {
    SocketLogText("C_Socket::InitListen: Socket already connected or initialized.");
    return false;
  }
  
  //get a socket
  m_hSock = socket(AF_INET, SOCK_STREAM, 0);
  if(m_hSock < 0) //if error, report and return errorcode
  {
    SocketLogText("C_Socket::InitListen: Error creating socket.");
    goto error;
  }
  
  //set it to reuse address, not important in windows
  setsockopt(m_hSock, SOL_SOCKET, SO_REUSEADDR, (const char*)&iReuseAddr, sizeof(iReuseAddr));
  
  //set socket non blocking
  if(!SetNonBlocking())
  {
    SocketLogText("C_Socket::InitListen: Error setting socket non blocking.");
    goto error;
  }
  
  //set the local address
  m_stAddrLocal.sin_family = PF_INET;
  m_stAddrLocal.sin_port = htons((unsigned short)i_iPort); //listening port
  m_stAddrLocal.sin_addr.s_addr = INADDR_ANY; //listen on all interfaces (maybe change later)
  
  //bind
  iRet = bind(m_hSock, (struct sockaddr*)&m_stAddrLocal, sizeof(struct sockaddr));
  if(iRet == SOCKET_ERROR)
  {
    OutputErrorMsg((char*)"C_Socket::InitListen: bind: ", WSAGetLastError());
    goto error;
  }
  
  //listen
  iRet = listen(m_hSock, SOMAXCONN); //changed from 4 to SOMAXCONN.
  if(iRet == SOCKET_ERROR)
  {
    OutputErrorMsg((char*)"C_Socket::InitListen: listen: ", WSAGetLastError());
    goto error;
  }
  
  iRet = getsockname(m_hSock, (struct sockaddr*)&stAddr, &iLen);
  if(iRet == SOCKET_ERROR)
  {
    OutputErrorMsg((char*)"C_Socket::InitListen: getsockname: ", WSAGetLastError());
  }
  else
  {
    m_iListenOnPort = ntohs(stAddr.sin_port);
  }
  
  m_bListening = true;
  return true;
error:
  closesocket(m_hSock);
  m_hSock = INVALID_SOCKET;
  return false;
}

bool C_Socket::Receive()
{
  int iRet, iLength;
  
  S_RecvData* pstData = new S_RecvData;
  
  iLength = SOCKET_BUFSIZE;
  unsigned int iBytes = 0;
  unsigned int iFlags = 0;
  
  if(!m_bUDP)
  {
    iRet = (int)recv(m_hSock, pstData->pcBuf, iLength, iFlags);
    pstData->iSrcIP = m_stAddrDest.sin_addr.s_addr;
    pstData->iSrcPort = ntohs(m_stAddrDest.sin_port);
  }
  else
  {
    socklen_t iStructLen = sizeof(m_stAddrDest);
    iRet = (int)recvfrom(m_hSock, pstData->pcBuf, iLength, iFlags, (struct sockaddr*)&m_stAddrDest, &iStructLen);
    pstData->iSrcIP = m_stAddrDest.sin_addr.s_addr;
    pstData->iSrcPort = ntohs(m_stAddrDest.sin_port);
  }
  
  bool bTerminate = false;
  if(iRet == SOCKET_ERROR)
  {
    int iErr = WSAGetLastError();
    if(iErr != WSAEWOULDBLOCK && iErr != WSAEWOULDBLOCK2)
    {
      OutputErrorMsg((char*)"C_Socket::Receive: recv/recvfrom: ", iErr);
      bTerminate = true;
    }
  }
  else
  {
    iBytes = iRet;
    if(iRet == 0 && !m_bUDP) bTerminate = true;
  }
  if(bTerminate)
  {
    SocketLogText("C_Socket::Receive: recv/recvfrom: socket terminated.");
    Disconnect();
  }
  pstData->iLength = iBytes;
  
#ifdef DEBUG_SOCKET
  FILE* pFile = fopen("net_rcv.txt", "ab");
  if(pFile)
  {
    char szIP[16];
    fprintf(pFile, "%s:%d (%d bytes)\r\n", C_Socket::IPToString(pstData->iSrcIP, szIP, sizeof(szIP)), pstData->iSrcPort, iBytes);
    fwrite(pstData->pcBuf, iBytes, 1, pFile);
    if(iBytes) fwrite("\r\n", 2, 1, pFile);
    fclose(pFile);
  }
#endif
  
  if(iBytes > 0)
  {
    m_pRcvPacketList->push_back(pstData);
    m_iBufferedReceive += iBytes;
    m_iTotByteRcvd += iBytes;
  }
  else
  {
    delete pstData;
  }
  return (iBytes != 0);
}

//sends data on a connected socket, always succeds if connected, data is copied and send later if socket buffer is full
int C_Socket::SendQueued(char* i_pData, int i_iLength)
{
  if(m_hSock == INVALID_SOCKET) return 0;
  
  int iResult = 0;
  if(m_clSendDataList.empty() && m_bConnected)   //queue empty, try to send directly
  {
    iResult = Send(i_pData, i_iLength);
  }
  
  //add unsent data to queue
  if(iResult != i_iLength)
  {
    int iNumBytes = i_iLength - iResult;
    S_SendData stData;
    stData.iLength = iNumBytes;
    stData.pData = new char[iNumBytes];
    memcpy(stData.pData, i_pData + iResult, iNumBytes);
    m_clSendDataList.push_back(stData);
    m_iBufferedSend += iNumBytes;
  }
  //try send queued data
  UpdateQueued();
  
  return i_iLength;
}

void C_Socket::UpdateQueued()
{
  if(m_hSock == INVALID_SOCKET || !m_bConnected) return;
  bool bSocketFull = false;
  while(!m_clSendDataList.empty() && !bSocketFull)
  {
    S_SendData* pstData = &m_clSendDataList[0];
    int iResult;
    iResult = Send(pstData->pData, pstData->iLength);
    if(iResult != pstData->iLength)
    {
      if(iResult > 0) //some data sent realloc memory
      {
        m_iBufferedSend -= iResult;
        int iNumBytes = pstData->iLength - iResult;
        char* pNewMem = new char[iNumBytes];
        memcpy(pNewMem, pstData->pData + iResult, iNumBytes);
        delete[] pstData->pData;
        pstData->iLength = iNumBytes;
        pstData->pData = pNewMem;
      }
      bSocketFull = true;
    }
    else     //all sent, delete and continue
    {
      m_iBufferedSend -= pstData->iLength;
      delete[] pstData->pData;
      m_clSendDataList.pop_front();
    }
  }
}

int C_Socket::Send(char* i_pData, int i_iLength)
{
  int iResult, iErr = 0;
  unsigned int iNumBytes = 0;
  
  if(m_hSock != INVALID_SOCKET)   //check if socket is valid
  {
    iResult = (int)send(m_hSock, i_pData, i_iLength, 0);
    if(iResult == SOCKET_ERROR) iErr = WSAGetLastError(); //save the error; must be done directly after
    else iNumBytes = iResult;
    
    //assert(iNumBytes==0 || iNumBytes==i_iLength);
    //^we assume that no part of the buffer can be sent, either all or nothing.
    //so if its wrong we have to detect it.
    //^commented the assert because if it fails the socket is probably dead
    //and the user will find out a little later...
    
#ifdef DEBUG_SOCKET
    if(iNumBytes != 0)
    {
      FILE* pFile = fopen("net_snd.txt", "ab");
      if(pFile)
      {
        char szIP[16];
        fprintf(pFile, "%s:%d (%d bytes)\r\n", C_Socket::IPToString((int)m_stAddrDest.sin_addr.s_addr, szIP, sizeof(szIP)), ntohs(m_stAddrDest.sin_port), iNumBytes);
        fwrite(i_pData, i_iLength, 1, pFile);
        if(iNumBytes) fwrite("\r\n", 2, 1, pFile);
        fclose(pFile);
      }
    }
#endif
    
    if(iResult == SOCKET_ERROR)
    {
      if(iErr != WSAEWOULDBLOCK && iErr != WSAEWOULDBLOCK2)
      {
        OutputErrorMsg((char*)"C_Socket::Send: send: ", iErr);
        //socket probably dead... do something here?
        return 0; //error
      }
    }
    //no error
    m_iTotByteSent += iNumBytes;
    return iNumBytes; //success
  }
  return 0; //error
}

C_Socket* C_Socket::AcceptConnection()
{
  socklen_t iLen = sizeof(struct sockaddr);
  C_Socket* pTCPSocket = new C_Socket();
  
  //reset
  m_bAcceptWillSucceed = false;
  
  //accept
  pTCPSocket->m_hSock = accept(m_hSock, (struct sockaddr*) &pTCPSocket->m_stAddrDest, &iLen);
  if(pTCPSocket->m_hSock == SOCKET_ERROR)
  {
    int iErr = WSAGetLastError();
    if(iErr != WSAEWOULDBLOCK && iErr != WSAEWOULDBLOCK2)
    {
      OutputErrorMsg((char*)"C_Socket::AcceptConnection: accept: ", iErr);
      goto error;
    }
  }
  
  //set socket non blocking
  if(!pTCPSocket->SetNonBlocking())
  {
    SocketLogText("C_Socket::AcceptConnection: Error setting socket non blocking.");
    goto error;
  }
  
  pTCPSocket->m_bConnected = true;
  
  return pTCPSocket;
error:
  delete pTCPSocket;
  return NULL;
}

int C_Socket::FlushRecvBuffer()
{
  if(!m_iBufferedReceive) return 0;
  
  size_t i, iSize = m_pRcvPacketList->size();
  for(i = 0; i < iSize; i++)
    delete(*m_pRcvPacketList)[i];
  m_pRcvPacketList->clear();
  int iRet = m_iBufferedReceive;
  m_iBufferedReceive = 0;
  
  return iRet;
}

bool C_Socket::GetRecvBuffer(char* o_pcBuf, int* io_iLength, int i_iTimeout, int* o_iIP, int* o_iPort) //returns true if buffer!=empty
{
  int iPartGet, iNumToGet = *io_iLength;
  *io_iLength = 0;
  
  //get available data (wait up to timeout)
  while(!m_iBufferedReceive || m_iBufferedReceive < iNumToGet)
  {
    if(!WaitForReadEvent(i_iTimeout)) break;
  }
  //if(!m_iBufferedReceive || m_iBufferedReceive<iNumToGet) WaitForReadEvent(i_iTimeout);
  
  //no data? fail.
  if(!m_iBufferedReceive || m_iBufferedReceive < iNumToGet)
  {
    return false;
  }
  
  while(*io_iLength != iNumToGet)
  {
    S_RecvData* pstData = m_pRcvPacketList->front();
    if(o_iIP) *o_iIP = pstData->iSrcIP;
    if(o_iPort) *o_iPort = pstData->iSrcPort;
    
    if(iNumToGet == -1) iNumToGet = pstData->iLength;
    iPartGet = pstData->iLength <= (iNumToGet - *io_iLength) ? pstData->iLength : (iNumToGet - *io_iLength);
    
    if(o_pcBuf) memcpy(o_pcBuf + (*io_iLength), pstData->pcBuf, iPartGet);
    *io_iLength += iPartGet;
    
    if(iPartGet == pstData->iLength)   //all is copied
    {
      m_pRcvPacketList->pop_front();
      delete pstData;
    }
    else
    {
      pstData->iLength -= iPartGet;
      memmove(pstData->pcBuf, pstData->pcBuf + iPartGet, pstData->iLength);
    }
  }
  m_iBufferedReceive -= iNumToGet;
  
  return true;
}

bool C_Socket::PeekRecvBuffer(char* o_pcBuf, int* io_iLength, int i_iTimeout, int* o_iIP, int* o_iPort)
{
  int iPartGet, iNumToGet = *io_iLength;
  *io_iLength = 0;
  
  //get available data (wait up to timeout)
  while(!m_iBufferedReceive || m_iBufferedReceive < iNumToGet)
  {
    if(!WaitForReadEvent(i_iTimeout)) break;
  }
  //if(!m_iBufferedReceive || m_iBufferedReceive<iNumToGet) WaitForReadEvent(i_iTimeout);
  
  //no data? fail.
  if(!m_iBufferedReceive || m_iBufferedReceive < iNumToGet)
  {
    return false;
  }
  
  int i = 0;
  while(*io_iLength != iNumToGet)
  {
    S_RecvData* pstData = (*m_pRcvPacketList)[i++];
    if(o_iIP) *o_iIP = pstData->iSrcIP;
    if(o_iPort) *o_iPort = pstData->iSrcPort;
    
    if(iNumToGet == -1) iNumToGet = pstData->iLength;
    iPartGet = pstData->iLength <= (iNumToGet - *io_iLength) ? pstData->iLength : (iNumToGet - *io_iLength);
    
    memcpy(o_pcBuf + (*io_iLength), pstData->pcBuf, iPartGet);
    *io_iLength += iPartGet;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
//UDP only, can be either send or recv

int C_Socket::InitRecvUDP(int i_iPort, bool i_bBroadcast)
{
  int iRet;
  sockaddr_in stAddr;
  socklen_t iLen = sizeof(stAddr);
  unsigned long iOne = 1;
  
  if(m_bListening) return false;
  if(m_hSock != INVALID_SOCKET) Disconnect(); //close socket if open
  ClearSendQueue();
  m_iListenOnPort = -1;
  
  //get a new socket
  m_hSock = socket(AF_INET, SOCK_DGRAM, 0);
  if(m_hSock < 0) //if error, report and return errorcode
  {
    SocketLogText("C_Socket::InitRecvUDP: Error creating socket.");
    goto error;
  }
  m_bUDP = true;
  
  if(i_bBroadcast)
  {
    //mark the socket for broadcast
    if(setsockopt(m_hSock, SOL_SOCKET, SO_BROADCAST, (const char*)&iOne, sizeof(iOne)) == SOCKET_ERROR)
    {
      SocketLogText("C_Socket::InitRecvUDP: Error setting send socket for broadcast.");
      goto error;
    }
  }
  
  //set the local address
  m_stAddrLocal.sin_family = AF_INET;                      //IP
  m_stAddrLocal.sin_port = htons((unsigned short)i_iPort); //source port (0 = any)
  m_stAddrLocal.sin_addr.s_addr = INADDR_ANY;              //any interface
  
  //bind
  iRet = bind(m_hSock, (struct sockaddr*)&m_stAddrLocal, sizeof(struct sockaddr));
  if(iRet == SOCKET_ERROR)
  {
    OutputErrorMsg((char*)"C_Socket::InitRecvUDP: bind: ", WSAGetLastError());
    goto error;
  }
  
  iRet = getsockname(m_hSock, (struct sockaddr*)&stAddr, &iLen);
  if(iRet == SOCKET_ERROR)
  {
    OutputErrorMsg((char*)"C_Socket::InitRecvUDP: getsockname: ", WSAGetLastError());
  }
  else
  {
    m_iListenOnPort = ntohs(stAddr.sin_port);
  }
  
  //set socket non blocking
  if(!SetNonBlocking())
  {
    SocketLogText("C_Socket::InitRecvUDP: Error setting socket non blocking.");
    goto error;
  }
  
  return m_iListenOnPort;
error:
  closesocket(m_hSock);
  m_hSock = INVALID_SOCKET;
  return -1;
}

//can use SendQueued() after this
bool C_Socket::InitSendUDP(int i_iAddr, int i_iPort, bool i_bBroadcast)
{
  int iRet;
  unsigned long iOne = 1;
  
  if(m_bListening) return false;
  if(m_hSock != INVALID_SOCKET) Disconnect(); //close socket if open
  ClearSendQueue();
  
  //get a new socket if non exists
  m_hSock = socket(AF_INET, SOCK_DGRAM, 0);
  if(m_hSock < 0) //if error, report and return errorcode
  {
    SocketLogText("C_Socket::InitSendUDP: Error creating socket.");
    goto error;
  }
  m_bUDP = true;
  
  if(i_bBroadcast)
  {
    //mark the socket for broadcast
    if(setsockopt(m_hSock, SOL_SOCKET, SO_BROADCAST, (const char*)&iOne, sizeof(iOne)) == SOCKET_ERROR)
    {
      SocketLogText("C_Socket::InitSendUDP: Error setting send socket for broadcast.");
      goto error;
    }
  }
  
  //set other address
  m_stAddrDest.sin_family = AF_INET;                      //IP
  m_stAddrDest.sin_port = htons((unsigned short)i_iPort); //init destination port
  m_stAddrDest.sin_addr.s_addr = i_iAddr;                 //init remote Address
  
  //set socket non blocking
  if(!SetNonBlocking())
  {
    SocketLogText("C_Socket::InitSendUDP: Error setting socket non blocking.");
    goto error;
  }
  
  //connect UDP socket
  iRet = connect(m_hSock, (struct sockaddr*)&m_stAddrDest, sizeof(struct sockaddr_in));
  if(iRet == SOCKET_ERROR)
  {
    int iErr = WSAGetLastError();
    if(iErr != WSAEWOULDBLOCK && iErr != WSAEWOULDBLOCK2)
    {
      OutputErrorMsg((char*)"C_Socket::InitSendUDP: connect:", iErr);
      goto error;
    }
  }
  
  m_bConnected = true;
  return true;
error:
  closesocket(m_hSock);
  m_hSock = INVALID_SOCKET;
  return false;
}


////////////////////////////////////////////////////////////////////////////////
//TCP and UDP common

void C_Socket::ClearSendQueue()
{
  size_t i, iSize = m_clSendDataList.size();
  for(i = 0; i < iSize; i++)
  {
    S_SendData* pstData = &m_clSendDataList[i];
    delete[] pstData->pData;
  }
  m_clSendDataList.clear();
}

//returns the destination ip, 0 if not connected (dead socket)
unsigned int C_Socket::ConnectedToIP()
{
  if(m_hSock == INVALID_SOCKET || !m_bConnected) return 0;
  return m_stAddrDest.sin_addr.s_addr;
}

bool C_Socket::WaitForReadEvent(int iTimeout)
{
  //check for read/write on socket
  if(m_hSock == INVALID_SOCKET)
  {
    SocketLogText("C_Socket::WaitForReadEvent: invalid socket\n");
    return false;
  }
  
  fd_set stReader;
  //clear all, set
  FD_ZERO(&stReader);
  FD_SET(m_hSock, &stReader);
  int iCount = 1;
#ifndef WIN32
  iCount = m_hSock + 1;
#endif
  
  UpdateQueued();
  
  //blocks until read event happens on the socket (when time!=0)
  timeval tv;
  tv.tv_sec  = iTimeout / 1000;
  tv.tv_usec = (iTimeout % 1000) * 1000;
  int iRet = select(iCount, &stReader, NULL, NULL, &tv);
  if(iRet < 0)
  {
    OutputErrorMsg((char*)"C_Socket::WaitForReadEvent: select: ", WSAGetLastError());
    return false; //error
  }
  
  if(iRet == 0)
  {
    //OutputErrorMsg((char*)"C_Socket::WaitForReadEvent: time out", 0);
    return false; //timeout
  }
  bool bReturn = false;
  if(FD_ISSET(m_hSock, &stReader))
  {
    if(m_bListening)
    {
      m_bAcceptWillSucceed = true;
      bReturn = true;
    }
    else
    {
      int iSafeCount = 100;
      while(Receive() && iSafeCount--) ;
      bReturn = true; //set true even if Receive() failed to recv anything, because we got an event from select
    }
  }
  return bReturn;
}

bool C_Socket::WaitForWriteEvent(int iTimeout)
{
  //check for read/write on socket
  if(m_hSock == INVALID_SOCKET) return false;
  
  fd_set stWriter;
  //clear all, set
  FD_ZERO(&stWriter);
  FD_SET(m_hSock, &stWriter);
  int iCount = 1;
#ifndef WIN32
  iCount = m_hSock + 1;
#endif
  
  UpdateQueued(); //try send queued
  
  //blocks until write event happens on the socket (when time!=0)
  timeval tv;
  tv.tv_sec  = iTimeout / 1000;
  tv.tv_usec = (iTimeout % 1000) * 1000;
  int iRet = select(iCount, NULL, &stWriter, NULL, &tv);
  if(iRet < 0)
  {
    OutputErrorMsg((char*)"C_Socket::WaitForWriteEvent: select: ", WSAGetLastError());
    return false; //error
  }
  if(iRet == 0)
  {
    return false; //timeout
  }
  if(!m_bListening && !m_bConnected)
  {
    m_bConnected = true;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
//static helper functions
//(only after WSAStartup or they may fail)

char* C_Socket::IPToString(int i_iIP, char* o_szResult, int i_iSize)
{
  if(!o_szResult || i_iSize < 16) return NULL;
  
  unsigned char* pIP = (unsigned char*)&i_iIP;
  
  sprintf(o_szResult, "%d.%d.%d.%d", pIP[0], pIP[1], pIP[2], pIP[3]);
  return o_szResult;
}

int C_Socket::ResolveAddress(const char* i_szHost)
{
  struct hostent* he;
  unsigned int iRet;
  
  iRet = inet_addr(i_szHost); //first try it as aaa.bbb.ccc.ddd
  if(iRet == INADDR_NONE)
  {
    he = gethostbyname(i_szHost); //if that fails, do a DNS lookup
    if(he != NULL)
    {
      iRet = *(unsigned int*)he->h_addr;
    }
    else
    {
      iRet = INADDR_NONE; //if that also fails, return INADDR_NONE (0xffffffff)
    }
  }
  
  return (int)iRet;
}

bool C_Socket::GetLocalAddressInfo(S_InterfaceInfo* o_pstList, int* io_iMaxNum)
{
  if(!o_pstList || !io_iMaxNum || *io_iMaxNum < 1) return false;
  
  int iNum = 24;
  int iOutNum = *io_iMaxNum;
  *io_iMaxNum = 0;
  
  SOCKET sd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sd == SOCKET_ERROR)
  {
    SocketLogText("C_Socket::GetLocalAddressInfo: Error creating socket.");
    return false;
  }
  
#ifdef WIN32
  //a windows specific implementation:
  INTERFACE_INFO* pInfoList = new INTERFACE_INFO[iNum];
  unsigned long iBytesReturned;
  if(WSAIoctl(sd, SIO_GET_INTERFACE_LIST, 0, 0, pInfoList,
              sizeof(INTERFACE_INFO)*iNum, &iBytesReturned, 0, 0) == SOCKET_ERROR)
  {
    SocketLogText("C_Socket::GetLocalAddressInfo: Error getting interface info.");
    delete[] pInfoList;
    closesocket(sd);
    return false;
  }
  closesocket(sd);
  
  //create result
  iNum = iBytesReturned / sizeof(INTERFACE_INFO);
  if(iNum > iOutNum) iNum = iOutNum;
  for(int i = 0; i < iNum; i++)
  {
    sockaddr_in* pAddress;
    
    memset(&o_pstList[i], 0, sizeof(o_pstList[i]));
    pAddress = (sockaddr_in*) & (pInfoList[i].iiAddress);
    strncpy(o_pstList[i].szIP, inet_ntoa(pAddress->sin_addr), sizeof(o_pstList[i].szIP));
    o_pstList[i].szIP[sizeof(o_pstList[i].szIP) - 1] = 0;
    pAddress = (sockaddr_in*) & (pInfoList[i].iiBroadcastAddress);
    strncpy(o_pstList[i].szBroadcast, inet_ntoa(pAddress->sin_addr), sizeof(o_pstList[i].szBroadcast));
    o_pstList[i].szBroadcast[sizeof(o_pstList[i].szBroadcast) - 1] = 0;
    pAddress = (sockaddr_in*) & (pInfoList[i].iiNetmask);
    strncpy(o_pstList[i].szMask, inet_ntoa(pAddress->sin_addr), sizeof(o_pstList[i].szMask));
    o_pstList[i].szMask[sizeof(o_pstList[i].szMask) - 1] = 0;
    
    unsigned long nFlags = pInfoList[i].iiFlags;
    o_pstList[i].bActive = (nFlags & IFF_UP) != 0;
    o_pstList[i].bLoopBack = (nFlags & IFF_LOOPBACK) != 0;
    o_pstList[i].bBroadcast = (nFlags & IFF_BROADCAST) != 0;
  }
  delete[] pInfoList;
  *io_iMaxNum = iNum;
#else
  //a linux specific implementation:
  char          buf[2048];
  struct ifconf ifc;
  struct ifreq* ifr;
  int           iInterfaces;
  
  //query available interfaces
  ifc.ifc_len = sizeof(buf);
  ifc.ifc_buf = buf;
  if(ioctlsocket(sd, SIOCGIFCONF, &ifc) == SOCKET_ERROR)
  {
    SocketLogText("C_Socket::GetLocalAddressInfo: Error getting interface info.");
    closesocket(sd);
    return false;
  }
  
  //create result
  ifr         = ifc.ifc_req;
  iInterfaces = ifc.ifc_len / sizeof(struct ifreq);
  if(iNum > iInterfaces) iNum = iInterfaces;
  if(iNum > iOutNum) iNum = iOutNum;
  for(int i = 0; i < iNum; i++)
  {
    struct ifreq* pItem = &ifr[i];
    sockaddr_in* pAddress;
  
    memset(&o_pstList[i], 0, sizeof(o_pstList[i]));
    pAddress = (sockaddr_in*) & (pItem->ifr_addr);
    strncpy(o_pstList[i].szIP, inet_ntoa(pAddress->sin_addr), sizeof(o_pstList[i].szIP));
    o_pstList[i].szIP[sizeof(o_pstList[i].szIP) - 1] = 0;
  
    //get broadcast address
    if(ioctlsocket(sd, SIOCGIFBRDADDR, pItem) != SOCKET_ERROR)
    {
      pAddress = (sockaddr_in*) & (pItem->ifr_broadaddr);
      strncpy(o_pstList[i].szBroadcast, inet_ntoa(pAddress->sin_addr), sizeof(o_pstList[i].szBroadcast));
      o_pstList[i].szBroadcast[sizeof(o_pstList[i].szBroadcast) - 1] = 0;
    }
    //get netmask address
    if(ioctlsocket(sd, SIOCGIFNETMASK, pItem) != SOCKET_ERROR)
    {
      pAddress = (sockaddr_in*) & (pItem->ifr_addr); //ifr_netmask really, but it doesn't work on macos
      strncpy(o_pstList[i].szMask, inet_ntoa(pAddress->sin_addr), sizeof(o_pstList[i].szMask));
      o_pstList[i].szMask[sizeof(o_pstList[i].szMask) - 1] = 0;
    }
    //get flags
    if(ioctlsocket(sd, SIOCGIFFLAGS, pItem) != SOCKET_ERROR)
    {
      unsigned long nFlags = pItem->ifr_flags;
      o_pstList[i].bActive = (nFlags & IFF_UP) != 0;
      o_pstList[i].bLoopBack = (nFlags & IFF_LOOPBACK) != 0;
      o_pstList[i].bBroadcast = (nFlags & IFF_BROADCAST) != 0;
    }
  }
  closesocket(sd);
  *io_iMaxNum = iNum;
#endif
  return true;
}

bool C_Socket::IsLocalAddress(int i_iIP)
{
  S_InterfaceInfo astList[16];
  int i, iNum = 16;
  
  if(GetLocalAddressInfo(&astList[0], &iNum))
  {
    for(i = 0; i < iNum; i++)
    {
      if(ResolveAddress(astList[0].szIP) == i_iIP) return true;
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
//Error logging

void C_Socket::OutputErrorMsg(char* i_szText, int i_iErrorNr)
{
  char szString[2048];
  
#ifdef WIN32
  char szErrorMsg[1024]; //error message string
  szErrorMsg[0] = 0;
  FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS | FORMAT_MESSAGE_MAX_WIDTH_MASK,
                NULL, i_iErrorNr, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)szErrorMsg, sizeof(szErrorMsg), NULL);
                
  sprintf(szString, "%s %s (%08x)", i_szText, szErrorMsg, i_iErrorNr);
#else
  sprintf(szString, "%s (%08x)", i_szText, i_iErrorNr);
#endif
  
  SocketLogText(szString);
}
