/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:10:45
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */

// NOTE: Only modify function implementations and behavior below.
// All type/struct/enum/global variable definitions should remain in the header file.
#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

// --- Helper for gradual wheel speed in Lazy mode ---
static uint8_t lazyWheelSpeed = 0;
static unsigned long lastWheelUpdate = 0;
const unsigned long wheelUpdateInterval = 90; // ms between speed steps (was 40)
const uint8_t wheelStep = 2; // speed increment per step (was 6)

#define _is_print 1
#define _Test_print 0


Application_xxx Application_SmartRobotCarxxx0;
ApplicationFunctionSet Application_FunctionSet;

/*硬件设备成员对象序列*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;

DeviceDriverSet_Servo AppServo;

// --- Cat-like Mode Enum and Variable ---
enum CatMode {
  CatMode_Lazy,
  CatMode_Curious
};
static CatMode currentCatMode = CatMode_Lazy;

/*f(x) int */
static boolean
function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}


// ...existing code...

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  while (Serial.read() >= 0)
  {
    /*清空串口缓存...*/
  }
  delay(2000);
  Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
}


/*
  直线运动控制：
  direction：方向选择 前/后
  directionRecord：方向记录（作用于首次进入该函数时更新方向位置数据，即:yaw偏航）
  speed：输入速度 （0--255）
  Kp：位置误差放大比例常数项（提高位置回复状态的反映，输入时根据不同的运动工作模式进行修改）
  UpperLimit：最大输出控制量上限
*/
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw; //偏航
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }
  //if (en != directionRecord)
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }
  //加入比例常数Kp
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }
  if (direction == Forward) //前进
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
                                           /*direction_B*/ direction_just, /*speed_B*/ L, /*controlED*/ control_enable);
  }
  else if (direction == Backward) //后退
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
                                           /*direction_B*/ direction_back, /*speed_B*/ R, /*controlED*/ control_enable);
  }
}
/*
  运动控制:
  1# direction方向:前行（1）、后退（2）、 左前（3）、右前（4）、后左（5）、后右（6）
  2# speed速度(0--255)
*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;


    Kp = 2;
    UpperLimit = 180;

  switch (direction)
  {
  case /* constant-expression */
      Forward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //前进时进入方向位置逼近控制环处理
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }

    break;
  case /* constant-expression */ Backward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //后退时进入方向位置逼近控制环处理
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 2;
    }

    break;
  case /* constant-expression */ Left:
    /* code */
    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Right:
    /* code */
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftForward:
    /* code */
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftBackward:
    /* code */
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightForward:
    /* code */
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightBackward:
    /* code */
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ stop_it:
    /* code */
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    break;
  default:
    directionRecord = 10;
    break;
  }
}

static void delay_xxx(uint16_t _ms)
{
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}

/*
  避障功能
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void)
{
  // --- Cat-like Obstacle Avoidance Logic ---
  static boolean first_is = true;
  static unsigned long lazyModeStartTime = 0;
  static unsigned long lastPrintTime = 0;

  // Start timer if entering Lazy mode
  if (currentCatMode == CatMode_Lazy && lazyModeStartTime == 0) {
    lazyModeStartTime = millis();
  }
  // Reset timer if not in Lazy mode
  if (currentCatMode != CatMode_Lazy) {
    lazyModeStartTime = 0;
  }
  // Print timer every 2 seconds for debug
  if (currentCatMode == CatMode_Lazy && millis() - lastPrintTime > 2000) {
    Serial.print("[CatMode] Lazy mode duration: ");
    Serial.print((millis() - lazyModeStartTime) / 1000);
    Serial.println("s");
    lastPrintTime = millis();
  }

  // Debug: Print current cat mode
  if (_is_print) {
    Serial.print("[CatMode] Current mode: ");
    switch (currentCatMode) {
      case CatMode_Lazy:
        Serial.println("Lazy");
        break;
      case CatMode_Curious:
        Serial.println("Curious");
        break;
      default:
        Serial.println("Unknown");
        break;
    }
  }

  // Only run in obstacle avoidance mode
  if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode)
  {
    uint8_t switc_ctrl = 0;
    uint16_t get_Distance;

    // --- Safety: Stop if car is lifted ---
    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }


    static int headPos = 90;
    static int headDir = 1; // 1 = right, -1 = left
    static unsigned long lastHeadMove = 0;
    static unsigned long idleUntil = 0;
    const int headMin = 75; // step 3/15: smaller sweep range
    const int headMax = 105;
    const int headStep = 1;
    const unsigned long headMoveInterval = 2420 + random(0, 1200); // ms between steps, even slower and more random
    const unsigned long minIdle = 5200 + random(0, 2500); // ms, randomized (increased again)
    const unsigned long maxIdle = 11200 + random(0, 3500); // ms, randomized (increased again)

    // --- Camera pan variables (simulate if no camera hardware) ---
    static int camPos = 90;
    static int camDir = 1;
    const int camMin = 80; // step 2/15: smaller pan range
    const int camMax = 100;
    const int camStep = 1;
    const unsigned long camMoveInterval = 2820 + random(0, 1800); // ms between steps, much slower and more random (step 1/15)
    static unsigned long lastCamMove = 0;

    // Only look around if wheels are not moving (simulate with switc_ctrl == 0)
    // Step 6/15: Safety check - pause sweeps if obstacle detected
    uint16_t safeDistance = 0;
    static uint16_t lastStationaryDistance = 0;
    static unsigned long lastStationaryCheck = 0;
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&safeDistance);
    // Step 8/15: If servo is stationary and a big change in distance is detected, exit Lazy mode
    if (currentCatMode == CatMode_Lazy && millis() - lastStationaryCheck > 1200) {
      lastStationaryCheck = millis();
      int diff = (int)safeDistance - (int)lastStationaryDistance;
      if (abs(diff) > 18 && abs(diff) < 200) { // Only react to significant, not spurious, changes
        currentCatMode = CatMode_Curious;
      }
      lastStationaryDistance = safeDistance;
    }
    static unsigned long lastLookAround = 0;
    static bool lookAroundActive = false;
    static int lookAroundTarget = 90;
    if (currentCatMode == CatMode_Lazy && switc_ctrl == 0 && safeDistance > 20) {
      // Step 7/15: Occasionally do a small 'look around' movement
      if (!lookAroundActive && millis() - lastLookAround > 9000 + random(0, 8000)) {
        lookAroundActive = true;
        // Step 10/15: Bias look-around target away from center
        int offset = (random(0, 2) == 0 ? -1 : 1) * (8 + random(0, 8));
        lookAroundTarget = 90 + offset;
        if (lookAroundTarget == 90) lookAroundTarget += (random(0, 2) == 0 ? -3 : 3);
      }
      if (lookAroundActive) {
        static int currentServoPos = 90;
        if (currentServoPos < lookAroundTarget) currentServoPos++;
        else if (currentServoPos > lookAroundTarget) currentServoPos--;
        AppServo.DeviceDriverSet_Servo_control(currentServoPos);
        if (currentServoPos == lookAroundTarget) {
          delay_xxx(400 + random(0, 400));
          // Step 11/15: Add extra random delay after look-around
          delay_xxx(random(0, 700));
          lookAroundActive = false;
          lastLookAround = millis();
        }
      }
      if (idleUntil == 0) {
        idleUntil = millis() + random(minIdle, maxIdle);
        // Randomize initial look direction after each idle
        // Always start a full sweep from one end
        if (random(0, 2) == 0) {
          headDir = 1;
          headPos = headMin;
        } else {
          headDir = -1;
          headPos = headMax;
        }
        // Set a flag to indicate a sweep is in progress
        static bool sweeping = true;
        if (random(0, 5) == 0) { // 20% chance to skip sweep
          sweeping = false;
        } else {
          sweeping = true;
        }
        // Move servo one degree at a time toward headPos
        static int currentServoPos = 90;
        // Step 9/15: When idle, bias the servo a few degrees off center
        int idleBias = (random(0, 2) == 0) ? -4 : 4;
        if (headPos == 90) headPos = 90 + idleBias;
        if (currentServoPos != headPos) {
          if (currentServoPos < headPos) currentServoPos++;
          else if (currentServoPos > headPos) currentServoPos--;
          AppServo.DeviceDriverSet_Servo_control(currentServoPos);
        }
      }
      if (millis() < idleUntil) {
        // Idle: do nothing
      } else {
        static bool sweeping = true;
        if (sweeping && millis() - lastHeadMove > headMoveInterval) {
          int targetPos = headPos + headDir;
          if (targetPos >= headMax) {
            targetPos = headMax;
            headDir = -1;
            // Step 4/15: pause longer at right end
            idleUntil = millis() + random(minIdle + 1200, maxIdle + 2200);
          } else if (targetPos <= headMin) {
            targetPos = headMin;
            headDir = 1;
            // Step 4/15: pause longer at left end
            idleUntil = millis() + random(minIdle + 1200, maxIdle + 2200);
          }
          headPos = targetPos;
          AppServo.DeviceDriverSet_Servo_control(headPos);
          lastHeadMove = millis();
          // If we've reached an end, finish the sweep and pause
          if (headPos == headMin || headPos == headMax) {
            sweeping = false;
            // Step 12/15: Occasionally pause at a random off-center angle
            if (random(0, 3) == 0) { // ~33% chance
              int watchAngle = headMin + 4 + random(0, headMax - headMin - 8);
              AppServo.DeviceDriverSet_Servo_control(watchAngle);
              // Step 13/15: Camera also pans to a random off-center angle and pauses
              int camWatchAngle = camMin + 2 + random(0, camMax - camMin - 4);
              // If you have a camera pan servo, call it here:
              // AppCamera.DeviceDriverSet_CameraServo_control(camWatchAngle);
              // For now, just print for debug:
              if (_is_print) {
                Serial.print("[CatMode] Camera watch angle: ");
                Serial.println(camWatchAngle);
              }
              delay_xxx(1200 + random(0, 1800));
              // Step 14/15: Occasionally play a soft 'sniff' motion
              if (random(0, 2) == 0) { // 50% chance
                int sniffAngle = watchAngle + (random(0, 2) == 0 ? -3 : 3);
                AppServo.DeviceDriverSet_Servo_control(sniffAngle);
                delay_xxx(120 + random(0, 120));
                AppServo.DeviceDriverSet_Servo_control(watchAngle);
              }
              // Step 15/15: Occasionally play a soft 'ear twitch' (camera pan)
              if (random(0, 3) == 0) { // ~33% chance
                int twitchAngle = camWatchAngle + (random(0, 2) == 0 ? -2 : 2);
                // AppCamera.DeviceDriverSet_CameraServo_control(twitchAngle);
                // For now, just print for debug:
                if (_is_print) {
                  Serial.print("[CatMode] Camera ear twitch: ");
                  Serial.println(twitchAngle);
                }
                delay_xxx(80 + random(0, 80));
                // AppCamera.DeviceDriverSet_CameraServo_control(camWatchAngle);
              }
            }
          }
        }

        // Camera pan: move slowly in sync with head, but with different range/interval
        if (millis() - lastCamMove > camMoveInterval) {
          // Only move by one degree per update
          int camTarget = camPos + camDir;
          if (camTarget >= camMax) {
            camTarget = camMax;
            camDir = -1;
          } else if (camTarget <= camMin) {
            camTarget = camMin;
            camDir = 1;
          }
          camPos = camTarget;
          // If you have a camera pan servo, call it here:
          // Move camera servo one degree at a time toward camPos
          static int currentCamServoPos = 90;
          if (currentCamServoPos != camPos) {
            if (currentCamServoPos < camPos) currentCamServoPos++;
            else if (currentCamServoPos > camPos) currentCamServoPos--;
            // AppCamera.DeviceDriverSet_CameraServo_control(currentCamServoPos);
          }
          // For now, just print for debug:
          if (_is_print) {
            Serial.print("[CatMode] Camera pan: ");
            Serial.println(camPos);
          }
          lastCamMove = millis();
        }
      }
    } else {
      idleUntil = 0; // Reset idle if not in Lazy mode or wheels moving
    }
    // Center head on first entry only if not in Lazy mode
    if (first_is == true && currentCatMode != CatMode_Lazy) {
      AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
      first_is = false;
    }

    // --- Main forward scan ---
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);
    Serial.println(get_Distance);

    // --- If obstacle is very close, perform avoidance routine ---
    if (function_xxx(get_Distance, 0, 20))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);


      // --- Cat-like random head wiggle after stopping ---
      if (random(0, 100) < 40) { // 40% chance to wiggle
        int center = 90;
        int wiggle = 20;
        AppServo.DeviceDriverSet_Servo_control(center - wiggle);
        delay_xxx(80);
        AppServo.DeviceDriverSet_Servo_control(center + wiggle);
        delay_xxx(80);
        AppServo.DeviceDriverSet_Servo_control(center);
        delay_xxx(60);
      }

      // --- Cat-like random pause/sniff after wiggle ---
      if (random(0, 100) < 70) { // 70% chance to pause/sniff
        uint16_t sniff_time = 120 + random(0, 180); // 120-300 ms
        delay_xxx(sniff_time);
      }

      // --- Omnidirectional scan: check left, center, right ---
      for (int i = 1; i < 6; i += 2) // i = 1 (right), 3 (center), 5 (left)
      {
        AppServo.DeviceDriverSet_Servo_control(30 * i /*Position_angle*/);
        delay_xxx(1);
        AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);

        // --- If obstacle detected in this direction ---
        if (function_xxx(get_Distance, 0, 20))
        {
          ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
          if (5 == i)
          {
            // --- If all directions blocked: back up and pivot ---
            ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 150);
            delay_xxx(500);
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
            delay_xxx(50);
            first_is = true;
            break;
          }
        }
        else
        {
          // --- Found an open direction: turn and go ---
          switc_ctrl = 0;
          switch (i)
          {
          case 1:
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
            break;
          case 3:
            ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
            break;
          case 5:
            ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
            break;
          }
          delay_xxx(50);
          first_is = true;
          break;
        }
      }
    }
    else // Path is clear, move forward
    {
      // --- Cat-like rare 'zoomies' burst ---
      if (random(0, 100) < 8) { // ~8% chance for zoomies
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 255); // max speed
        delay_xxx(250 + random(0, 200)); // short burst 250-450ms
        // Gradually ramp down wheel speed to 0
        while (lazyWheelSpeed > 0) {
          unsigned long now = millis();
          if (now - lastWheelUpdate > wheelUpdateInterval) {
            lastWheelUpdate = now;
            if (lazyWheelSpeed > wheelStep) lazyWheelSpeed -= wheelStep;
            else lazyWheelSpeed = 0;
            ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, lazyWheelSpeed);
          }
        }
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        delay_xxx(60);
      } else {
        // Gradually ramp up to 150 in Lazy mode
        unsigned long now = millis();
        if (now - lastWheelUpdate > wheelUpdateInterval) {
          lastWheelUpdate = now;
          if (lazyWheelSpeed < 150) lazyWheelSpeed += wheelStep;
          if (lazyWheelSpeed > 150) lazyWheelSpeed = 150;
        }
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, lazyWheelSpeed);
      }
    }
  }
  else
  {
    // --- Reset on mode exit ---
    first_is = true;
  }
}
