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

#include "WorldNetworkClient.h"


WorldNetworkClient::WorldNetworkClient()
:WorldInterface(){
}
WorldNetworkClient::~WorldNetworkClient(){
}

WorldInterface::Status WorldNetworkClient::WorldInit(){
    // Initialize the network    
    pXmlTree options = GetOptionTree();
    if(options){
        mNetwork.Init(options->Get("Options.ServerName",string("127.0.0.1")).c_str(),
                      options->Get("Options.ServerPort",1207),false);
        mNetwork.SetTimers(0,5000);
    }else{
        mNetwork.Init("127.0.0.1",1207,false);
        mNetwork.SetTimers(0,1000);
    }

    return STATUS_OK;
}
WorldInterface::Status WorldNetworkClient::WorldFree(){
    mNetwork.Disconnect();
    return STATUS_OK;
}
WorldInterface::Status WorldNetworkClient::WorldStart(){
    return STATUS_OK;
}    
WorldInterface::Status WorldNetworkClient::WorldStop(){
    return STATUS_OK;
}
WorldInterface::Status WorldNetworkClient::WorldUpdate(){
    
    // Update the network and if a message is received,
    //  sets the world accordingly
    mNetwork.Step();
    if(mNetwork.GetMessage()>0){
        char* dataPtr = mNetwork.GetIncomingMessageBuffer();
        dataPtr  += GetWorld()->SetFromStream(dataPtr);
    }

    return STATUS_OK;
}
WorldInterface::Status WorldNetworkClient::WorldUpdateCore(){
    return STATUS_OK;
}
int WorldNetworkClient::RespondToCommand(const string cmd, const vector<string> &args){
    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    WorldNetworkClient* create(){return new WorldNetworkClient();}
    void destroy(WorldNetworkClient* module){delete module;}
}

