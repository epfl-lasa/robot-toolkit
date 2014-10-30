/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Klas Kronander
 * email:   klas.kronander@epfl.ch
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




void GolfModule::learn(int num_k){
  //choose number of gaussians 
  adjustData.Resize(demoCount,4,true);
  //adjustData*=1000;
  adjustModel.initEM_kmeans(num_k,adjustData);
  //      adjustModel.debug();
  cout <<"******************"<<endl;
  adjustModel.doEM(adjustData,0,100);
  Vector incomp(2);
  Vector outcomp(2);
  incomp(0)=0;
  incomp(1)=1;
  outcomp(0)=2;
  outcomp(1)=3;
  adjustModel.inComponents=incomp;
  adjustModel.outComponents=outcomp;

}

void GolfModule::adjust(){
  Vector res(2);
  Vector inp(2);
  inp(0)=Target_pos(0)-hitting_point(0);
  inp(1)=Target_pos(1)-hitting_point(1);
  res=adjustModel.doRegression(inp);
  hitting_gain=res(0);
  beta=res(1);
  changeHittingDirection();

}


void GolfModule::addCurrent(){
  //package the new input
  Vector newData(4);
  //  newData.SetSubVector(0,Target_pos-Ball_pos);
  newData(0)=Target_pos(0)-hitting_point(0);
  newData(1)=Target_pos(1)-hitting_point(1);
  newData(2)=hitting_gain;
  newData(3)=beta;
  //add the new input;
  if(adjustData.RowSize()<=demoCount)
    adjustData.Resize(demoCount+1,adjustData.ColumnSize(),true);
  adjustData.SetRow(newData,demoCount);
  demoCount++;
  cout<<"current demonstration set:"<<endl;
  adjustData.Print();
  cout<<demoCount<<endl;
}

void GolfModule::loadTrainingData(const char filename[],int democnt){
  char fme[256];
  sprintf(fme,"./src/packages/GolfPackage/data/Misc/GolfModule/Data/%s",filename);
  if(adjustData.Load(fme)){
    GetConsole()->Print("succesfully loaded data set");
    demoCount=democnt;
    cout<<demoCount<<democnt<<endl;
  }
  else
    GetConsole()->Print("failed to load data set");
  adjustData.Print();
}

void GolfModule::saveTrainingData(const char filename[]){
  char fname[256];
  sprintf(fname,"./src/packages/GolfPackage/data/Misc/GolfModule/Data/%s",filename);
  if(adjustData.Save(fname)){
    GetConsole()->Print("succesfully saved data set");
  }
  else
    GetConsole()->Print("failed to save data set");
}

void GolfModule::clearTrainingData(){
  adjustData.Zero();
  demoCount=0;
}

void GolfModule::saveModel(const char filename[]){
  adjustModel.saveParams(filename);

}
