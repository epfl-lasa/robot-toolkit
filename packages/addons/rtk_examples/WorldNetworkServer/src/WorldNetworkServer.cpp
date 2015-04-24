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

#include "WorldNetworkServer.h"


WorldNetworkServer::WorldNetworkServer()
:WorldInterface(){
}
WorldNetworkServer::~WorldNetworkServer(){
}

WorldInterface::Status WorldNetworkServer::WorldInit(){
    // Initialize the network
    pXmlTree options = GetOptionTree();
    if(options){
        mNetwork.Init(options->Get("Options.ClientName",string("127.0.0.1")).c_str(),
                      options->Get("Options.ClientPort",1207),true);
        mNetwork.SetTimers(0,1000);
    }else{
        mNetwork.Init("127.0.0.1",1207,true);
        mNetwork.SetTimers(0,1000);
    }
    
    if(GetConsole()){
        AddConsoleCommand("connect");
        AddConsoleCommand("h");
	}
    return STATUS_OK;
}
WorldInterface::Status WorldNetworkServer::WorldFree(){
    mNetwork.Disconnect();
    return STATUS_OK;
}
WorldInterface::Status WorldNetworkServer::WorldStart(){
    return STATUS_OK;
}    
WorldInterface::Status WorldNetworkServer::WorldStop(){
    return STATUS_OK;
}
WorldInterface::Status WorldNetworkServer::WorldUpdate(){

    // Serialize the world and send it through the network
    int size = GetWorld()->SetStream(mNetwork.GetOutgoingMessageBuffer());
    mNetwork.SendMessage(size);
    mNetwork.Step();
    return STATUS_OK;
}
WorldInterface::Status WorldNetworkServer::WorldUpdateCore(){
    return STATUS_OK;
}

int WorldNetworkServer::RespondToConsoleCommand(const string cmd, const vector<string> &args){	
    if(GetConsole()){
	    if(cmd=="connect" && args.size()==2){
			mNetwork.Init(args[0].c_str(),atoi(args[1].c_str()),true);
		}else if(cmd=="h"){
			GetConsole()->Print("<server> Defining the server ip");
		}
	}
    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    WorldNetworkServer* create(){return new WorldNetworkServer();}
    void destroy(WorldNetworkServer* module){delete module;}
}

