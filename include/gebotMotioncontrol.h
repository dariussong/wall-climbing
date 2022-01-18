#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <stdio.h>
#include <stdlib.h>
#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Camera.hpp>
#include <webots/Emitter.hpp>

using namespace std;
using namespace Eigen;
using namespace webots;
namespace webots{
  class PositionSensor;
  class Motor;
  class TouchSensor;
  class InertialUnit;
}
class CreepMotionControl  // 格式：变量首字母小写；单词分割采用cmdVel或cmd_vel；类首字母大写；全局变量全部大写；注释采用“a=1  // 注释”的格式，即代码后两个空格+注释符+一个空格；
{
    public:
        //Vector<float, 3> cmdV;  // command velocity: X Y alpha
        float timeGait;  // The time of the whole period
        float presentTime;
        float times;
        //float timePeriod;  // The time of one period
        Vector<float, 4> comPos;  // position of center of mass
        Vector<float, 4> comPos_bymotor;
        Matrix<float, 4, 4> shoulderPos;  // LF RF LH RH X, Y in world
        Matrix<float, 4, 4> shoulderPos_bymotor;
        Matrix<float, 4 ,3> footPos;  // LF RF LH RH X, Y, Z
        Matrix<float, 4 ,3> ftsPos;  // LF RF LH RH X, Y, Z foot_to_shoulder
        Matrix<float, 4 ,3> ftsPos_bymotor;
        Vector<float, 4> targetCoMVelocity;  // X, Y , alpha c in world cordinate
        //Matrix<float, 4, 3> shoulderPos_for_real;  // real shoulder position by motor feedback
        //Vector<float, 4> com_position;  // coordinate of center of mass
        //Vector<float, 4> com_position_for_real;
       

        Matrix<float, 4 ,3> motorPos;  // LF RF LH RH 1,2,3 motor position command
        Matrix<float, 4 ,3> motorPosFdb;  // LF RF LH RH 1,2,3 motor position command by motor feedback
        Matrix<float, 4, 2> timeForStancePhase;  // startTime, endTime: LF, RF, LH, RH
        Matrix<float, 4, 2> timeForSwingPhase;
        Vector<float, 4> swingTime;
        Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
        Vector<bool, 4> stanceFlag;  // True, False: LF, RF, LH, RH
        float L1, L2, L3;  // The length of L
        float width, length;

        Matrix<float, 4, 3> stancePhaseStartPos;
        Matrix<float, 4, 3> stancePhaseEndPos;
        Matrix<float, 4, 3> legPresentPos;  // present X-Y-Z: LF, RF, LH, RH in CoM coordinate
        Matrix<float, 4, 3> legCmdPos;  // command X-Y-Z: LF, RF, LH, RH in CoM coordinate
        Matrix<float, 4, 3> jointPresentPos;  // present motor 0-11
       // Matrix<float, 4, 3> jointCmdPos;  // command motor 0-11

        //void setInitPos(Matrix<float, 4, 3> initPosition);
        void setCoMVel(Vector<float, 4> tCV);
        void nextStep();
        void inverseKinematics();
        void forwardKinematics();
        void setJointPosition();
        void initiation(Robot *robot);
        void setInitPos();
        void sensorUpdate();
        char positionName[12][22] = {"LF0 PositionSensor", "LF1 PositionSensor", "LF2 PositionSensor", "RF0 PositionSensor", "RF1 PositionSensor", "RF2 PositionSensor", "LH0 PositionSensor", "LH1 PositionSensor", "LH2 PositionSensor", "RH0 PositionSensor", "RH1 PositionSensor", "RH2 PositionSensor"};
        char motorName[12][22] = {"LF0 RotationalMotor", "LF1 RotationalMotor", "LF2 RotationalMotor", "RF0 RotationalMotor", "RF1 RotationalMotor", "RF2 RotationalMotor", "LH0 RotationalMotor", "LH1 RotationalMotor", "LH2 RotationalMotor", "RH0 RotationalMotor", "RH1 RotationalMotor", "RH2 RotationalMotor"};
        char touchsensorName[4][16] = {"LF_touch_sensor", "RF_touch_sensor", "LH_touch_sensor", "RH_touch_sensor"};
        PositionSensor *Ps[12];
        Motor *Tor[12];
        TouchSensor *Ts[4];
        InertialUnit *imu;
        float jointCmdPos[12];  // command motor 0-11
        double jointPos[12];
        float jointCmdPosLast[12];
        float jointCmdVel[12];
        float motorTorque[1];
        float motorInitPos[12];   // init motor angle of quadruped state
        float pid_motortorque[12];
        float jacobian_motortorque[12]; //motor torque in VMC
        float motorCmdTorque[12];
        Vector<float, 3> imu_num;        //VMC
};