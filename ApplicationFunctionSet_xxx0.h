/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-19 15:46:13
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */

#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <arduino.h>

// Enums for motion and functional model
enum SmartRobotCarMotionControl {
  Forward,
  Backward,
  Left,
  Right,
  LeftForward,
  LeftBackward,
  RightForward,
  RightBackward,
  stop_it
};

enum SmartRobotCarFunctionalModel {
  Standby_mode,
  TraceBased_mode,
  ObstacleAvoidance_mode,
  Follow_mode,
  Rocker_mode
};

// Struct for application state
struct Application_xxx {
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};

extern Application_xxx Application_SmartRobotCarxxx0;

class ApplicationFunctionSet
{
public:
  void ApplicationFunctionSet_Init(void);
  void ApplicationFunctionSet_Obstacle(void);           //避障
  
private:
  volatile uint16_t UltrasoundData_mm; //超声波数据
  volatile uint16_t UltrasoundData_cm; //超声波数据
  boolean UltrasoundDetectionStatus = false;
public:
  boolean Car_LeaveTheGround = true;
  const int ObstacleDetection = 20;
};
extern ApplicationFunctionSet Application_FunctionSet;

#endif
