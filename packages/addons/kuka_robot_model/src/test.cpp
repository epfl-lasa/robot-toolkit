#include "LWRRobot.h"


int main(void){
  
  LWRRobot hejsan;
  hejsan.GetJointStiffness().Print();

  LWRRobot * pRobot;
  pRobot = new LWRRobot();

  return 1;
}


