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

#include "UDPNetwork.h"
#include <iostream>
using namespace std;

#ifdef WIN32
#pragma warning( disable : 4996)
#endif

//#define IFDEBUG_PINGPONG(x) {x}
//#define IFDEBUG_TIMING(x)   {x}

#define IFDEBUG_PINGPONG(x)
#define IFDEBUG_TIMING(x)

#define TIMESYNC_NUMPASS 53

UDPNetwork::UDPNetwork(){
  Server = NULL;
  Client = NULL;
  Server = new UDPConnection();
  Client = new UDPConnection();

  clientHasConnected  = false;
  serverHasConnected  = false;
  connectionError     = false;
  bIsPinging          = false;
  bIsClient           = false;
  bMsgToSend          = false;
  bMsgRecevied        = false;
  bAutoFlush          = true;
  bSingleShootFlush   = false;
  bAutoSyncTime       = false;
  bSyncTimeBlocking   = false;

  recvMsgSize = 0;
  sentMsgSize = 0;
  DeltaT      = 0.1;
  PingDeltaT  = 1.0;
  PingTimeout = 1.0;

  sprintf(remotePCName,"localhost");
  port  = 1207;

  for(int i=0;i<MAX_ALLOWED_CLIENT;i++)
    allowedClients[i][0] = 0;

  mNetworkTime.Start();
  mNetworkTimeOffset        = 0.0;
  mNetworkTimeCurrDelta     = 0.0;
  mRequestNetworkTimeState  = 0;
  mNetworkTimeMeanLatency   = 0;
  mNetworkTimeCurrLatency   = 0;

  double timeNow = mNetworkTime.ElapsedTime();
  UDPTimeToGo    = timeNow + DeltaT;

  bVerboseMessages = true;
}

UDPNetwork::~UDPNetwork(){
  Free();
}


void  UDPNetwork::Free(){
  if(Server !=NULL) delete Server; Server = NULL;
  if(Client !=NULL) delete Client; Client = NULL;
}

void  UDPNetwork::Reset(){
  if(Server != NULL) delete Server;
  if(Client != NULL) delete Client;
  Server = new UDPConnection();
  Client = new UDPConnection();

  Server->ClearAllowedClients();

  if(bIsClient){
    Server->InitServer(port+1);
  }else{
    Server->InitServer(port);
  }

  Server->AddAllowedClient(remotePCName);

  if(!bIsClient){
  //Server->AddAllowedClient(127,0,0,1);
    if(bVerboseMessages){
      cerr << "UDPNetwork: <Server> Waiting for a client on port "<< port <<endl;
      cerr  <<"UDPNetwork: <Server> Allowed client are:"<<endl;
      cerr << "UDPNetwork: <Server>   - "<< remotePCName <<endl;
    }
    for(int i=1;i<MAX_ALLOWED_CLIENT;i++){
        if(allowedClients[i][0] != 0){
            //cout << i<<" "<<allowedClients[i]<<endl;
            Server->AddAllowedClient(allowedClients[i]);
            if(bVerboseMessages){
              cerr << "UDPNetwork: <Server>   - "<< allowedClients[i] <<endl;
            }
        }
    }
  }



  if(bIsClient){
    Client->InitClient(remotePCName,port);
    if(bVerboseMessages)
      cerr << "UDPNetwork: <Client> Looking for server at "<<remotePCName<<":"<<port<<endl;
  }
  //else
    //Client->InitClient(remotePCName,port+1);

  //UDPTimer.Start(DeltaT);
  double timeNow = mNetworkTime.ElapsedTime();
  UDPTimeToGo    = timeNow + DeltaT;

  clientHasConnected  = false;
  serverHasConnected  = false;
  connectionError     = false;
  bIsPinging          = false;
  bMsgToSend          = false;
  bMsgRecevied        = false;
  bAutoFlush          = true;
  recvMsgSize         = 0;
  sentMsgSize         = 0;

  mRequestNetworkTimeState  = 0;
  mNetworkTimeMeanLatency   = 0;
  mNetworkTimeCurrLatency   = 0;
}

void  UDPNetwork::ShowConnectionMessage(bool verbose){
    bVerboseMessages = verbose;
}

void  UDPNetwork::AddAllowedClient(const char *clientName){
    int id = -1;
    for(int i=1;i<MAX_ALLOWED_CLIENT;i++){
        if(allowedClients[i][0] == 0){ id = i; break;}
    }
    if(id>0){
        strncpy(allowedClients[id],clientName,255);
    }
}

void  UDPNetwork::Init(const char *remotePC, int portNo, bool server){
  port = portNo;
  strcpy(remotePCName,remotePC);
  bIsClient = !server;
  Reset();
}

void  UDPNetwork::SetTimers(int poolTimeMs, int testTimeoutMs){
  if(poolTimeMs<0)    poolTimeMs    = 100;
  if(testTimeoutMs<0) testTimeoutMs = 1000;

  DeltaT      = double(poolTimeMs)*0.001;
  PingDeltaT  = double(testTimeoutMs)*0.001;
  PingTimeout = double(testTimeoutMs)*0.001;
}

void  UDPNetwork::SetAutoFlush(bool bAuto){
  bAutoFlush = bAuto;
}
void  UDPNetwork::SetAutoSyncTime(bool bAuto){
    bAutoSyncTime     = bAuto;
}
void  UDPNetwork::SetSyncTimeBlocking(bool bBlocking){
    bSyncTimeBlocking = bBlocking;
}
int   UDPNetwork::Step(){

    do{

    double timeNow = mNetworkTime.ElapsedTime();

    bMsgRecevied = false;

    if((DeltaT==0.0) || (timeNow > UDPTimeToGo)){
        if(connectionError){
            if(bVerboseMessages)
                cerr<< "UDPNetwork: "<<(!bIsClient?"<Server>":"<Client>") <<" Connection Error: Reseting..."<<endl;
            Reset();
        }

        if((!clientHasConnected)||(!serverHasConnected)){
            //Connection phase
            if(!serverHasConnected){
                Server->WaitForClient(0);
                if(Server->IsConnected()){
                    serverHasConnected = true;
                    if(clientHasConnected){
                        if(bVerboseMessages)
                            cerr<< "UDPNetwork: "<<(!bIsClient?"<Server>":"<Client>") <<" Connected..."<<endl;
                        if(bAutoSyncTime) RequestNeworkTime();
                        UDPPingTimeToGo = timeNow + PingDeltaT;
                    }else{
                        if(!Client->IsInitialized()){
                            if(bVerboseMessages)
                                cerr<< "UDPNetwork: <Server> Received request from "<<Server->GetClientIP()<<":" <<port<<endl;
                            Client->InitClient(Server->GetClientIP(),port+1);
                        }
                    }
                }
            }
            if(!clientHasConnected){
                Client->ConnectToServer(0);
                if(Client->IsConnected()){
                    clientHasConnected = true;
                    if(serverHasConnected){
                        if(bAutoSyncTime) RequestNeworkTime();
                        if(bVerboseMessages)
                            cerr<< "UDPNetwork: "<<(!bIsClient?"<Server>":"<Client>") <<" Connected..."<<endl;
                        UDPPingTimeToGo = timeNow + PingDeltaT;
                    }
                }
            }

        }else if((Server->IsConnected())&&(Client->IsConnected())){

            // Checking pong
            if(Client->HasPong()){
                IFDEBUG_PINGPONG(cout<<"Got Pong"<<endl;);
                // Self acknoledge of the message
                bIsPinging = false;
                // Got something so the other is alive
                UDPPingTimeToGo = timeNow + PingDeltaT;

                // If in time requesting mode
                if(mRequestNetworkTimeState>=2){
                    // Retreive timing
                    memcpy(&mRequestTimeStruct,Client->GetPingPongBuffer(),sizeof(RequestTimeStruct));
                    if(mRequestNetworkTimeState>6){
                        double currLatency       = (0.5*(mRequestTimeStruct.timeDelta)+0.5*(timeNow-mRequestTimeStruct.timeHere));
                        mNetworkTimeCurrLatency += 0.5*currLatency;
                        mNetworkTimeCurrDelta   += mRequestTimeStruct.timeThere + 0.5*currLatency - timeNow;
                        mNetworkTimeOffset       = mNetworkTimeCurrDelta   / (double(mRequestNetworkTimeState-6));
                        mNetworkTimeMeanLatency  = mNetworkTimeCurrLatency / (double(mRequestNetworkTimeState-6));
                        IFDEBUG_TIMING(cout<<"Now computing DeltaT "<< endl;);
                    }
                    mRequestTimeStruct.timeHere = timeNow;
                    mRequestTimeStruct.seqId    = mRequestNetworkTimeState;

                    if(mRequestNetworkTimeState < TIMESYNC_NUMPASS){
                        if(Server->Ping(&mRequestTimeStruct,sizeof(mRequestTimeStruct))){
                            IFDEBUG_TIMING(cout<<"Sent "<< mRequestTimeStruct.seqId <<" "<<mRequestTimeStruct.timeHere<<" "<< mRequestTimeStruct.timeThere<< " "<<mRequestTimeStruct.timeDelta<<endl;);
                            bIsPinging      = true;
                            UDPPongTimeToGo = timeNow + PingTimeout;
                            mRequestNetworkTimeState ++;
                        }else{
                            connectionError = true;
                        }
                    }else{
                        mRequestNetworkTimeState = 0;
                        if(bVerboseMessages){
                            cerr<< "UDPNetwork: "<<(!bIsClient?"<Server>":"<Client>") <<" Network time request completed..."<<endl;
                        }
                    }
                    //UDPPongChrono.Start();
                }
                //UDPPingChrono.Start();
            }

            // Checking time request
            if(mRequestNetworkTimeState==1){
                if(!bIsPinging){
                    mRequestTimeStruct.seqId      =  1;
                    mRequestTimeStruct.timeHere   =  timeNow;
                    mRequestTimeStruct.timeThere  = -1.0;
                    mRequestTimeStruct.timeDelta  = -1.0;
                    mNetworkTimeCurrDelta         =  0.0;
                    mNetworkTimeCurrLatency       =  0.0;

                    if(Server->Ping(&mRequestTimeStruct,sizeof(mRequestTimeStruct))){

                        if(bVerboseMessages){
                            cerr<< "UDPNetwork: "<<(!bIsClient?"<Server>":"<Client>") <<" Sending network time request..."<<endl;
                        }

                        IFDEBUG_TIMING(cout<<"Sent "<< mRequestTimeStruct.seqId <<" "<<mRequestTimeStruct.timeHere<<" "<< mRequestTimeStruct.timeThere<< " "<<mRequestTimeStruct.timeDelta<<endl;);
                        bIsPinging      = true;
                        UDPPongTimeToGo = timeNow + PingTimeout;
                        mRequestNetworkTimeState = 2;
                    }else{
                        connectionError = true;
                    }
                }
            }


            // If not news from some time, ask for ping-pong
            if(timeNow > UDPPingTimeToGo){
                if(!bIsPinging){
                    mRequestTimeStruct.seqId = 0;
                    if(Server->Ping(&mRequestTimeStruct,sizeof(mRequestTimeStruct))){
                        IFDEBUG_PINGPONG(cout<<"Ping"<<endl;);
                        bIsPinging      = true;
                        UDPPongTimeToGo = timeNow + PingTimeout;
                    }else{
                        connectionError = true;
                    }
                }
            }

            // If ping-pong request and no news... error
            if((bIsPinging)&&(timeNow > UDPPongTimeToGo)){
                connectionError = true;
            }

            // Recieved a ping request
            if(Client->HasPing()){
                IFDEBUG_PINGPONG(cout<<"Got Ping"<<endl;);
                // Got something so the other is alive
                UDPPingTimeToGo = timeNow + PingDeltaT;

                // Prepare pong message
                memcpy(&mRequestTimeStruct,Client->GetPingPongBuffer(),sizeof(RequestTimeStruct));
                if(mRequestTimeStruct.seqId>0){
                    if(mRequestTimeStruct.timeThere>=0.0){
                        mRequestTimeStruct.timeDelta = timeNow - mRequestTimeStruct.timeThere;
                    }
                    if(mRequestTimeStruct.seqId<TIMESYNC_NUMPASS-1)
                        mRequestNetworkTimeState = -1;
                    else
                        mRequestNetworkTimeState = 0;

                    //IFDEBUG_TIMING(cout<<"Sent "<< mRequestTimeStruct.seqId <<" "<<mRequestTimeStruct.timeHere<<" "<< mRequestTimeStruct.timeThere<< " "<<mRequestTimeStruct.timeDelta<<endl;);
                    mRequestTimeStruct.timeThere = timeNow;

                    if(bVerboseMessages){
                        if(mRequestTimeStruct.seqId==1){
                            cerr<< "UDPNetwork: "<<(!bIsClient?"<Server>":"<Client>") <<" Received network time request..."<<endl;
                        }
                        if(mRequestTimeStruct.seqId==TIMESYNC_NUMPASS-1){
                            cerr<< "UDPNetwork: "<<(!bIsClient?"<Server>":"<Client>") <<" Network time request completed..."<<endl;
                        }
                    }

                    IFDEBUG_TIMING(cout<<"Sent <"<< mRequestNetworkTimeState <<">"<< mRequestTimeStruct.seqId <<" "<<mRequestTimeStruct.timeHere<<" "<< mRequestTimeStruct.timeThere<< " "<<mRequestTimeStruct.timeDelta<<endl;);

                    // send pong message
                    if(!Server->Pong(&mRequestTimeStruct,sizeof(mRequestTimeStruct))){
                        IFDEBUG_PINGPONG(cout<<"Pong failed"<<endl;);
                        connectionError = true;
                    }
                }else{
                    // send pong message
                    if(!Server->Pong()){
                        IFDEBUG_PINGPONG(cout<<"Pong failed"<<endl;);
                        connectionError = true;
                    }
                }
            }

            // Receiving messages (I think bAutoFlush=true is a good practice :)
            int mSize;
            if((mSize = Client->GetData(recvMsg,MAX_BUFFER_SIZE,0,(bAutoFlush)||(bSingleShootFlush)))>0){
                recvMsgSize = mSize;
                bMsgRecevied = true;

                // Got something so the other is alive
                UDPPingTimeToGo = timeNow + PingDeltaT;
            }
            // Have something to send?
            if(bMsgToSend){
                bMsgToSend = false;
                if(!(Server->SendData(msgToSend,sentMsgSize))){
                    connectionError = true;
                }
            }


        }else{
            connectionError = true;
        }

        UDPTimeToGo = timeNow + DeltaT;
    }
    //if((bSyncTimeBlocking && (!RequestNeworkTimeDone())&&(IsConnected())))
        //cout<< "IS SYNCING"<<endl;
    }while(bSyncTimeBlocking && (!RequestNeworkTimeDone())&&(IsConnected()));
    return IsConnected();
}

bool  UDPNetwork::IsConnected(){
  return (!(connectionError||(!clientHasConnected)||(!serverHasConnected)));
}

void  UDPNetwork::Flush(){
  bSingleShootFlush = true;
}


int   UDPNetwork::GetMessage(char *msg, int maxSize){
  if(bMsgRecevied){
    if(msg !=NULL){
      int size = (maxSize>recvMsgSize?recvMsgSize:maxSize);
      memcpy(msg,recvMsg,size);
    }
    return recvMsgSize;
  }
  return 0;
}

void  UDPNetwork::SendMessage(const char *msg, int size){
  if(msg !=NULL){
    int _size = (size>MAX_BUFFER_SIZE?MAX_BUFFER_SIZE:size);
    if(_size!=size){
        cerr << "Warning: message to be sent is bigger than MAX_BUFFER_SIZE("<<MAX_BUFFER_SIZE<<" bytes). Truncating message"<<endl;
    }
    memcpy(msgToSend,msg,_size);
    sentMsgSize = _size;
    bMsgToSend = true;
  }
}
char* UDPNetwork::GetOutgoingMessageBuffer(){
  return msgToSend;
}
char* UDPNetwork::GetIncomingMessageBuffer(){
  return recvMsg;
}
int   UDPNetwork::GetMessageMaxSize(){
  return MAX_BUFFER_SIZE;
}

int   UDPNetwork::GetMessage(){
  if(bMsgRecevied){
    return recvMsgSize;
  }
  return 0;
}
void  UDPNetwork::SendMessage(int size){
  int _size = (size>MAX_BUFFER_SIZE?MAX_BUFFER_SIZE:size);
  sentMsgSize = _size;
  bMsgToSend = true;
}

void  UDPNetwork::RequestNeworkTime(){
    mRequestNetworkTimeState = 1;
}
const Chrono& UDPNetwork::GetNetworkTimer(){
    return mNetworkTime;
}
double UDPNetwork::GetNetworkTime(){
    return mNetworkTime.ElapsedTime() + mNetworkTimeOffset;
}
double UDPNetwork::GetNetworkTimeOffset(){
    return mNetworkTimeOffset;
}
bool  UDPNetwork::RequestNeworkTimeDone(){
    return (mRequestNetworkTimeState==0);
}
double UDPNetwork::GetNetworkLatency(){
    return mNetworkTimeMeanLatency;
}

void  UDPNetwork::Disconnect(){
  if(Server != NULL) Server->CloseConnection();
  if(Client != NULL) Client->CloseConnection();
  clientHasConnected  = false;
  serverHasConnected  = false;
  connectionError     = false;
  bIsPinging          = false;
  bMsgToSend          = false;
  bMsgRecevied        = false;
  bAutoFlush          = true;
  recvMsgSize         = 0;
  sentMsgSize         = 0;

}
