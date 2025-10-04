#pragma once

#include "id.hpp"

struct PowerBoard_Target
{
  bool ON_OFF;
};

struct PowerBoard_Status
{
  float Current;
  float output_voltage;
  float battery1_voltage;
  float battery2_voltage;
  bool emengecy_on;
};


struct PowerBoard_format {
  ID id;
  union 
  {
    PowerBoard_Status status;
    PowerBoard_Target target;
    const char data[64];
  } data;
};