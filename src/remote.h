/*
   -- SelfBalancingRobotControll --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 2.4.3 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.5.1 or later version;
     - for iOS 1.4.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_BLE

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "SelfBalancingRobot"


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,31,0,119,0,40,2,10,13,4,
  131,1,1,1,14,5,1,2,31,67,
  111,110,116,114,111,108,108,0,131,0,
  16,1,14,5,2,2,31,83,116,97,
  116,117,115,0,66,132,61,15,19,15,
  1,2,24,129,0,2,12,5,4,3,
  17,107,112,0,129,0,3,20,3,4,
  3,17,107,105,0,129,0,2,28,5,
  4,3,17,107,100,0,7,44,59,12,
  13,5,3,2,26,2,2,7,44,59,
  20,13,5,3,2,26,2,2,7,44,
  59,28,13,5,3,2,26,2,2,65,
  7,88,26,9,9,1,4,128,8,11,
  49,5,3,2,26,4,128,8,19,49,
  5,3,2,26,2,0,69,48,15,5,
  1,2,26,31,31,79,78,0,79,70,
  70,0,66,131,80,2,17,8,1,2,
  24,131,0,54,1,15,5,3,2,31,
  80,73,68,0,2,0,68,40,16,5,
  1,2,26,31,31,79,78,0,79,70,
  70,0,129,0,54,49,10,3,1,17,
  77,111,116,111,114,115,0,4,128,9,
  56,56,5,3,2,26,129,0,2,53,
  8,3,3,17,77,111,116,111,114,32,
  112,111,119,101,114,32,108,105,109,105,
  116,0,5,1,2,11,48,48,1,2,
  26,31,129,0,54,41,12,3,1,17,
  66,97,108,97,110,99,101,0,67,4,
  78,56,20,5,3,2,13,11,65,7,
  88,13,9,9,1,1,0,87,49,12,
  12,1,2,31,67,65,76,0,67,4,
  48,56,38,5,1,2,26,21,131,0,
  31,1,13,5,4,2,31,83,112,101,
  101,100,0,68,51,4,15,93,45,2,
  8,36,135,94,101,114,114,111,114,0,
  97,110,103,108,101,0,116,97,114,103,
  101,116,65,110,103,108,101,0,68,51,
  4,15,93,45,4,8,36,135,94,80,
  87,77,100,117,116,121,0,82,80,83,
  0,116,97,114,103,101,116,32,82,80,
  83,0,4,128,8,27,49,5,3,2,
  26,67,6,82,12,16,5,3,2,13,
  11,67,6,82,20,16,5,3,2,13,
  11,67,6,82,28,16,5,3,2,13,
  11,129,0,2,36,5,4,3,17,107,
  112,0,7,44,59,36,13,5,3,2,
  26,2,2,4,128,8,36,49,5,3,
  2,26,67,6,82,36,16,5,3,2,
  13,11,129,0,2,43,5,4,3,17,
  107,105,0,7,44,59,43,13,5,3,
  2,26,2,2,4,128,8,43,49,5,
  3,2,26,67,6,82,43,16,5,3,
  2,13,11,129,0,2,8,5,2,3,
  17,80,105,100,32,65,110,103,108,101,
  0,129,0,2,33,10,2,3,17,80,
  105,100,32,83,112,101,101,100,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  float pidKpEdit;
  float pidKiEdit;
  float pidKdEdit;
  int8_t pidKp; // =0..100 slider position 
  int8_t pidKi; // =0..100 slider position 
  uint8_t motorsOn; // =1 if switch ON and =0 if OFF 
  uint8_t pidOn; // =1 if switch ON and =0 if OFF 
  int8_t motorLimit; // =0..100 slider position 
  int8_t joystickA_x; // =-100..100 x-coordinate joystick position 
  int8_t joystickA_y; // =-100..100 y-coordinate joystick position 
  uint8_t buttonCalibrate; // =1 if button pressed, else =0 
  int8_t pidKd; // =0..100 slider position 
  float pidSpeedKpEdit;
  int8_t pidSpeedKp; // =0..100 slider position 
  float pidSpeedKiEdit;
  int8_t pidSpeedKi; // =0..100 slider position 

    // output variables
  int8_t angle; // =0..100 level position 
  uint8_t ledState_r; // =0..255 LED Red brightness 
  uint8_t ledState_g; // =0..255 LED Green brightness 
  uint8_t ledState_b; // =0..255 LED Blue brightness 
  int8_t speed; // =0..100 level position 
  char motorLimitOut[11];  // string UTF8 end zero 
  uint8_t ledBallance_r; // =0..255 LED Red brightness 
  uint8_t ledBallance_g; // =0..255 LED Green brightness 
  uint8_t ledBallance_b; // =0..255 LED Blue brightness 
  char txtCalibrate[21];  // string UTF8 end zero 
  float graph_var1;
  float graph_var2;
  float graph_var3;
  float speedGraph_var1;
  float speedGraph_var2;
  float speedGraph_var3;
  char pidKpVal[11];  // string UTF8 end zero 
  char pidKiVal[11];  // string UTF8 end zero 
  char pidKdVal[11];  // string UTF8 end zero 
  char pidSpeedKpVal[11];  // string UTF8 end zero 
  char pidSpeedKiVal[11];  // string UTF8 end zero 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////
