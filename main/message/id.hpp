#pragma once

#include <cstdint>

enum class Message_Type
{
  Target,
  Status,
  Config,
  EMENGECY,
};

enum class Board_Type
{
  Master_Board,
  PowerBoard,
  MotorBoard
};

union ID
{
  struct ID_format {
    int to_BoardID : 4;
    Board_Type to_BoardType : 4;
    
    int from_BoardID : 4;
    Board_Type from_BoardType: 4;

    Message_Type message_type : 2;
    bool broadcast : 1;
  } format;
  uint16_t id = 0; 
};

