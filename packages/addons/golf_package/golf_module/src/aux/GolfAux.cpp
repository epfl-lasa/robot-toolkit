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

void GolfModule::exploreModeDo(){

  //update the distance from ball to hole
  final_dist=(Ball_pos.GetSubVector(0,2)-Target_pos.GetSubVector(0,2)).Norm();
  //if this distance is smaller than min_dist, update min_dist
  if (final_dist<min_dist)
    min_dist=final_dist;

  //final_dist represents the final distance between ball and hole, and the min_dist represents the minimum distance seen through the trajectory. 


  //   cout<<min_dist<<endl;



  if (eventID==2 && GetClock().GetTime()-exploreclock  > 4)
    {
      //update target pos and hitting params
      Target_pos(0)=exploreTargetPos(exploreTargetIter,0);
      Target_pos(1)=exploreTargetPos(exploreTargetIter,1);
      updateTargetPosition();
      beta=DEG2RAD(exploreParams(exploreParamIter,0));
      hitting_gain=exploreParams(exploreParamIter,1)*1.2/100;
      
      //      cout<<beta<<" "<<hitting_gain<<endl;
      
      //cout<<exploreParamIter<<" "<<exploreTargetIter<<endl;  
      exploreParamIter++;


      if(exploreParamIter==exploreParams.RowSize()){
	cout<<"Changing Target positon"<<endl;
	
	exploreParamIter=0;
	exploreTargetIter++;
	cout<<exploreTargetIter<<"/"<<exploreTargetPos.RowSize()-1<<endl;
      }

      

      
      changeHittingDirection();//apply changes to hitting direction
      
      exploreclock=GetClock().GetTime();//start timing
      
      //      record=true;//record
      mState=3;//start hitting motion
      min_dist=10;
      final_dist=10;
    }

  if (eventID==5 && GetClock().GetTime()-exploreclock  > 1)
  {
    mState=1;
    rw=true;
  }

  if (rw && GetClock().GetTime()-exploreclock > 2)
  {
    rw=false;

    ofstream outfile(exploreFileName.c_str(),ios::app);

    outfile<<Target_pos(0)<<" "<<Target_pos(1)<<" "<<beta<<" "<<hitting_gain<<" "<<final_dist<<" "<<min_dist<<endl;
    //stop and save recording
    // record=false;
    // if(record1)
    //   saveRecording();
    // if(record2)
    //   saveRecording2();
    // mDataLogCount++;
    //save the data in smaller 'package'
    //    resetWorld();//put ball back in initial position
    if(exploreTargetIter==exploreTargetPos.RowSize()){
      exploreMode=false;
      cout<<"finished exploring!"<<endl;
    }

  }
}





void GolfModule::changeHittingDirection(){
  //set default hitting direction
  hitting_dir =Default_dir; // reset hitting dir
  //rotate
  R.Identity();
  R(0,0)=cos(beta+angular_offset);
  R(0,1)=-sin(beta+angular_offset);
  R(1,0)=sin(beta+angular_offset);
  R(1,1)=cos(beta+angular_offset);
  //hitting_dir is only used in deterministic correction of end effector orientation. Thus, it need not be aligned with the DS. 
  hitting_dir = R*hitting_dir;
}

// void GolfModule::resetWorld(){
//   if(mRobot->GetWorld()->mMainWorldInterface){

//     if(dynamic_cast<SimulatorDynamicsInterface*>(mRobot->GetWorld()->mMainWorldInterface)){
//       dynamic_cast<SimulatorDynamicsInterface*>(mRobot->GetWorld()->mMainWorldInterface)->mSimulatorDynamics.AllowRobotReset(false);
//     }
//     mRobot->GetWorld()->mMainWorldInterface->Stop();
//     mRobot->GetWorld()->mMainWorldInterface->Start();
//   }


// }









// void GolfModule::useAsDemo(){//method to add the current settings to the adjustData matrix.

//   //package the new input
//   Vector newData(4);
//   //  newData.SetSubVector(0,Target_pos-Ball_pos);
//   newData(0)=Target_pos(0)-Ball_pos(0);
//   newData(1)=Target_pos(1)-Ball_pos(1);
//   newData(2)=beta;
//   newData(3)=hitting_gain; 
//   //add the new input;
//   adjustData.SetRow(newData,demoCount);
//   demoCount++;
 
// }


Vector GolfModule:: load_FilterCoefficients(const char fileName[]){//load filter specified in fileName
  Vector B;
  int order;
  std::ifstream fich(fileName);
  if (fich.is_open())
    {
      fich >>order;
      B.Resize(order+1);
      for(int i=0; i<=order+1;i++){
	fich>>B(i);
      }
    }
  else
    {
      cout<<"could not load filter file!"<<endl;
      
    }
  
  return B;

}

void GolfModule::Filter(Vector& input, Matrix& input_old,int& inputp,Vector& filter){
  //first, add the new values to input Matrix:

  input_old.SetRow(input,inputp);

  int temp=filter.Size();  

  //do the filtering
  input*=0;
  int ind;
  ind=inputp;
  int count;
  count=0;
  while(count!=temp){

    input_old.GetRow(ind,TmpRowFiltering);
    TmpRowFiltering.ScaleAddTo(filter(count),input);
    //input+=input_old.GetRow(ind)*filter(count);
  
    if(ind==0)
      ind=temp-1;
    else
      ind--;

    count++;

  }
  inputp++;
  if(inputp==temp)
    inputp=0;
}



// computing a rotation matrix that maps vector Racket_dirto next_dir
Matrix GolfModule::Compute_Rotation_Matrix(Vector Racket_dir, Vector next_dir){
  Vector w_rotation(3); //Rotation Vector
  Matrix uuT(3,3); //u*u^T
  Matrix Su(3,3); // Skew Symmetric form of u
  Matrix I(3,3); //identity Matrix
	
  double alpha = acos(Racket_dir.Dot(next_dir)/Racket_dir.Norm()/next_dir.Norm()); //angle of rotation
	
  Vector u(3); //rotation axis
	
  u[0]  = Racket_dir[1] * next_dir[2] - Racket_dir[2] * next_dir[1];
  u[1]  = Racket_dir[2] * next_dir[0] - Racket_dir[0] * next_dir[2];
  u[2]  = Racket_dir[0] * next_dir[1] - Racket_dir[1] * next_dir[0];

  if (u.Norm()!=0)
    u /= u.Norm();
	
  // computing u*u^T
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      uuT(i,j)=u[i]*u[j];

  //Computing the skew symmetric form of u
  Su(0,1) = -u[2];Su(1,0) =  u[2];
  Su(0,2) =  u[1];Su(2,0) = -u[1];
  Su(1,2) = -u[0];Su(2,1) =  u[0];

	
  //computing a rotation matrix that maps vector Racket_dirto next_dir
  Matrix R = uuT + (I.Identity() - uuT)*cos(alpha) + Su*sin(alpha);
	
  return R;
}


void GolfModule::updateTargetPosition(){
  pWorldObject obj = GetWorld()->Find("hole");
  Vector3 Target_pos3;
  Target_pos3(0)=Target_pos(0);
  Target_pos3(1)=Target_pos(1);
  Target_pos3(2)=Target_pos(2);
  obj->GetReferenceFrame().SetOrigin(Target_pos3);
}
