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
  { 255,25,0,52,0,97,1,10,13,4,
  131,1,1,1,22,7,1,2,31,67,
  111,110,116,114,111,108,108,0,131,0,
  26,1,20,7,2,2,31,83,116,97,
  116,117,115,0,68,19,4,15,93,45,
  2,8,36,135,94,66,132,90,3,8,
  6,0,2,24,129,0,3,15,5,4,
  3,17,107,112,0,129,0,4,24,3,
  4,3,17,107,105,0,129,0,3,32,
  5,4,3,17,107,100,0,7,44,74,
  15,20,5,3,2,26,2,2,7,44,
  74,23,20,5,3,2,26,2,2,7,
  44,74,31,20,5,3,2,26,2,2,
  65,7,87,52,9,9,1,4,128,9,
  15,63,5,3,2,26,4,128,9,23,
  63,5,3,2,26,4,128,9,31,63,
  5,3,2,26,2,0,14,55,15,5,
  1,2,26,31,31,79,78,0,79,70,
  70,0,66,131,71,2,17,8,0,2,
  24,131,0,48,1,20,7,3,2,31,
  80,73,68,0,2,0,40,55,16,5,
  1,2,26,31,31,79,78,0,79,70,
  70,0,129,0,3,56,10,3,1,17,
  77,111,116,111,114,115,0,4,128,9,
  52,56,5,3,2,26,129,0,3,48,
  8,3,3,17,77,111,116,111,114,32,
  112,111,119,101,114,32,108,105,109,105,
  116,0,5,1,8,15,30,30,1,2,
  26,31,129,0,34,56,5,3,1,17,
  80,73,68,0,67,4,74,51,20,5,
  3,2,26,11,65,7,87,38,9,9,
  1,1,0,85,13,12,12,1,2,31,
  67,65,76,0,67,4,45,16,38,5,
  1,2,26,21,7,44,74,38,20,5,
  3,2,26,2,2,129,0,3,39,8,
  3,3,17,116,97,114,103,101,116,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  float pidKpEdit;
  float pidKiEdit;
  float pidKdEdit;
  int8_t pidKp; // =0..100 slider position 
  int8_t pidKi; // =0..100 slider position 
  int8_t pidKd; // =0..100 slider position 
  uint8_t motorsOn; // =1 if switch ON and =0 if OFF 
  uint8_t pidOn; // =1 if switch ON and =0 if OFF 
  int8_t motorLimit; // =0..100 slider position 
  int8_t joystickA_x; // =-100..100 x-coordinate joystick position 
  int8_t joystickA_y; // =-100..100 y-coordinate joystick position 
  uint8_t buttonCalibrate; // =1 if button pressed, else =0 
  float target;

    // output variables
  float graph_var1;
  float graph_var2;
  float graph_var3;
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

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
///////////////////////////////////////////// 