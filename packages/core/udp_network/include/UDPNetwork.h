/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __UDPNetwork_H__
#define __UDPNetwork_H__

/* #if defined(WIN32) || defined (USE_FLAT_SOURCE_CODE) */
/* #include <StdTools/Timer.h> */
/* #include "Timer.h" */
/* #else */
/* #include <StdTools/Timer.h> */
/* #include "StdTools/Timer.h" */
/* #endif */
#include "Timer.h"
#include "UDPConnection.h"



class UDPNetwork{

public:
  UDPConnection *Server;
  UDPConnection *Client;

  Chrono mNetworkTime;
  double mNetworkTimeOffset;
  int    mRequestNetworkTimeState;
  double mNetworkTimeCurrDelta;
  double mNetworkTimeCurrLatency;
  double mNetworkTimeMeanLatency;

  //Timer  UDPTimer;
  double UDPTimeToGo;

  double UDPPingTimeToGo;
  double UDPPongTimeToGo;
  //Chrono UDPPingChrono;
  //Chrono UDPPongChrono;

  bool clientHasConnected;
  bool serverHasConnected;
  bool connectionError;
  bool bIsPinging;

  double DeltaT;
  double PingDeltaT;
  double PingTimeout;

  bool bIsClient;

  char remotePCName[256];
  char allowedClients[MAX_ALLOWED_CLIENT][256];
  int  port;

  char msgToSend[MAX_BUFFER_SIZE];
  char recvMsg[MAX_BUFFER_SIZE];
  bool bMsgToSend;
  bool bMsgRecevied;
  int  recvMsgSize;
  int  sentMsgSize;

  bool bAutoFlush;
  bool bSingleShootFlush;
  bool bAutoSyncTime;
  bool bSyncTimeBlocking;

  bool bVerboseMessages;
public:
  UDPNetwork();
  ~UDPNetwork();


  void  Free();

  void  Reset();

  void  Init(const char *remotePC, int portNo, bool server=true);
  void  SetTimers(int poolTimeMs, int testTimeoutMs);
  int   Step();
  bool  IsConnected();
  void  Disconnect();

  void  AddAllowedClient(const char *clientName);
  void  SetAutoFlush(bool bAuto);
  void  SetAutoSyncTime(bool bAuto);
  void  SetSyncTimeBlocking(bool bBlocking);
  void  Flush();

  int   GetMessage(char *msg, int maxSize);
  void  SendMessage(const char *msg, int size);

  void  ShowConnectionMessage(bool verbose);

  char* GetOutgoingMessageBuffer();
  char* GetIncomingMessageBuffer();
  int   GetMessageMaxSize();

  int   GetMessage();
  void  SendMessage(int size);

  void  RequestNeworkTime();
  bool  RequestNeworkTimeDone();
   const Chrono& GetNetworkTimer();
   double GetNetworkTime();
   double GetNetworkTimeOffset();
   double GetNetworkLatency();

  typedef struct{
    int     seqId;
    int     dummy;
    double  timeHere;
    double  timeThere;
    double  timeDelta;
  } RequestTimeStruct;
  RequestTimeStruct     mRequestTimeStruct;
};

typedef UDPNetwork *pUDPNetwork;

#endif
