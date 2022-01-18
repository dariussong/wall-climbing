// File:          gebotController.cpp
// Date:
// Description:
// Author:
// Modifications:


#include <webots/Robot.hpp>
#include <gebotMotioncontrol.h>
#include <ctime>
#include <webots/Keyboard.hpp>
#define PI 3.1415926
using namespace webots;
CreepMotionControl mc;
int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Keyboard kb;
  kb.enable(1000);
  // int key;
  int timeStep = (int)robot->getBasicTimeStep(); 
  mc.initiation(robot);  
  mc.inverseKinematics();
  mc.setInitPos();  // set initposition
  // for(int i = 0; i<4; i++){
//       for(int j = 0; j<3; j++){
//             cout<<"ibss="<<mc.ftsPos(i,j)<<endl;
//             cout<<"ibss="<<mc.motorPos(i,j)<<endl;
//       }
//   }
  // cout<<"ibss"<<endl;
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
    Emitter *emitter;
    emitter = robot->getEmitter("emitter"); 
  //emitter->enable(TIME_STEP);
  // mc.initiation();
    Vector<float, 4> tCV;
    tCV<< 6.0, 0.0, 0.0, 0.0;
    mc.setCoMVel(tCV);
  while (robot->step(timeStep) != -1)
  {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    mc.nextStep();
    mc.inverseKinematics();
    mc.setJointPosition();
    mc.sensorUpdate();
    mc.forwardKinematics();

    cout<<"presentTime="<<mc.presentTime<<endl;
    cout<<"times="<<mc.times<<endl;
    // for(int i=0; i<4; i++){
    //     for(int j=0; j<3; j++){
    //         cout<<"ftsPos("<<i<<","<<j<<")="<<mc.ftsPos(i,j)<<endl;
    //     }
    // }
    // cout<<"comPos(0)="<<mc.comPos(0)<<endl;
    // cout<<"comPos_bymotor(0)="<<mc.comPos_bymotor(0)<<endl;
    // cout<<"footPos(1,0)="<<mc.footPos(1,0)<<endl;
    // cout<<"stancePhaseStartPos(1,0)="<<mc.stancePhaseStartPos(1,0)<<endl;
    // cout<<"stancePhaseEndPos(1,0)="<<mc.stancePhaseEndPos(1,0)<<endl;
    // cout<<"timeForSwingPhase(legNum, 0)="<<mc.timeForSwingPhase(0, 0)<<endl;
    // cout<<"stancePhaseStartPos(1,0)="<<mc.stancePhaseStartPos(1,0)<<endl;
    // cout<<"stancePhaseStartPos(2,0)="<<mc.stancePhaseStartPos(2,0)<<endl;
    // cout<<"stancePhaseStartPos(3,0)="<<mc.stancePhaseStartPos(3,0)<<endl;
    //  cout<<"ftsPos(1,0)="<<mc.ftsPos(1,0)<<endl;
    //  cout<<"ftsPos(1,2)="<<mc.ftsPos(1,2)<<endl;
    //  cout<<"motorPos(0,0)="<<mc.motorPos(0,0)<<endl;
    //  cout<<"motorPos(0,1)="<<mc.motorPos(0,1)<<endl;
    //  cout<<"motorPosFdb(0,0)="<<mc.motorPosFdb(0,0)<<endl;
    //  cout<<"motorPosFdb(0,1)="<<mc.motorPosFdb(0,1)<<endl;
    float temp_send[4][3];
    for(int i=0; i<4; i++)
    {
      for(int j=0; j<3; j++)
      {
        temp_send[i][j]=mc.Ts[i]->getValues()[j];
        cout << "Ts["<<i<<"]["<<j<<"]="<<temp_send[i][j]<<endl;
      }
    }  // *getvalue &get location
   // cout << "Ts1 = "<<temp_send[0][0]<<endl;//<<" Ts2 = "<< temp_send[1]<<" Ts3 = "<< temp_send[2]<<" Ts4 = "<<temp_send[3]<<endl;
    float timeSend[2];
    timeSend[0] = mc.presentTime;
    emitter->send(timeSend, sizeof(timeSend));
    // Enter here functions to send actuator commands, like:
    // motor->setPosition(10.0);
  }

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
