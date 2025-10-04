#pragma once 

#include "id.hpp"

enum class ControlMode
{
  PWM_Mode,
  Encoder_Mode,
  Current_Mode,      
};

struct MotorBoard_Status
{
  ControlMode mode;
  bool target;
  float current;
};

struct MotorBoard_Target
{
  ControlMode modem;
  int target[4];
};

struct MotorBoard_format {
  ID id;
  union 
  {
    MotorBoard_Status status;
    MotorBoard_Target target;
    const char data[64];
  } data;
};