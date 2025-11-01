#pragma once

#include <string>
#include <vector>
#include <functional>
#include <unordered_map>

enum Commands
{
  INVALID,
  START,
  STOP,
  SLOWER,
  FASTER,
  WAVE,
  HANDOVERDROP,
  TASK_START,
  TAKEBAG_YES,
  TAKEBAG_NO,
  TAKEBAG_DONE,
};

namespace commands
{
const std::string MSG_OP_START = "go ahead";
const std::string MSG_OP_STOP = "please wait";
const std::string MSG_OP_SLOWER = "move slower";
const std::string MSG_OP_FASTER = "move faster";
const std::string MSG_OP_HANDOVER = "here is your bag";
const std::string MSG_OP_TASK_START = "Hello human, I will carry your luggage";
const std::string MSG_OP_WAVE = "please wave for me";

const std::string MSG_OP_TAKEBAG_YES = "i will pickup the bag";
const std::string MSG_OP_TAKEBAG_DONE = "I am ready to go";
const std::string MSG_OP_TAKEBAG_NO = "i cant pickup the bag";

const std::string MSG_RO_BAG = "can you pick up the bag";
const std::string MSG_RO_TARGET_REACHED = "we arrived";
const std::string MSG_RO_START = "please follow me";
const std::string MSG_RO_RETURN = "thanks you can go back now";

static std::unordered_map<std::string, Commands> const COMMANDS_TYPE_MAP = {
  { "invalid", Commands::INVALID },
  { MSG_OP_START, Commands::START },
  { MSG_OP_STOP, Commands::STOP },
  { MSG_OP_SLOWER, Commands::SLOWER },
  { MSG_OP_FASTER, Commands::FASTER },
  { MSG_OP_WAVE, Commands::WAVE },
  { MSG_OP_HANDOVER, Commands::HANDOVERDROP },
  { MSG_OP_TASK_START, Commands::TASK_START },
  { MSG_OP_TAKEBAG_YES, Commands::TAKEBAG_YES },
  { MSG_OP_TAKEBAG_DONE, Commands::TAKEBAG_DONE },
  { MSG_OP_TAKEBAG_NO, Commands::TAKEBAG_NO },
};
static Commands StringToCommand(const std::string& str)
{
  auto it = COMMANDS_TYPE_MAP.find(str);
  if (it != COMMANDS_TYPE_MAP.end())
  {
    return it->second;
  }

  return Commands::INVALID;
};
}  // namespace commands
