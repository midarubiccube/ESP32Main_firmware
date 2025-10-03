#pragma once

#include <cstdint>

union ID
{
  struct ID_format {
    int message_type : 4;
    
    int from_id1 : 2;
    int from_id2 : 4;
    int from_type : 4;

    int to_id1 : 2;
    int to_id2 : 4;
    int to_type : 4;
  } format;
  uint32_t id; 
};

struct PowerBoard_receive
{
  bool ON_OFF;
  uint8_t LED_R;
  uint8_t LED_G;
  uint8_t LED_B;
  float MAX_current;
  bool LED_FULLColor;
};

struct PowerBoard_send
{
  float Current;
  float output_voltage;
  float battery1_voltage;
  float battery2_voltage;
  bool emengecy_on;
};


struct MotorBoard 
{
  int mode;
  int target[4];
};

struct MotorBoard_send
{
  uint8_t mode;
  bool target;
  float current;
};


struct Message_format {
  ID id;
  bool is_remote = false;
  union{
    PowerBoard_receive power_rsv;
    PowerBoard_send power_send;
    MotorBoard_receive motor_rsv;
    MotorBoard_send  motor_send;
    char data[64] = {0};
  } data;
};