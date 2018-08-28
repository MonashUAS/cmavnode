#ifndef BLOCKXMIT_H
#define BLOCKXMIT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <iterator>
#include <iomanip>
#include <chrono>
#include <thread>
#include <boost/format.hpp>
#include <string>
#include "../include/mavlink2/ardupilotmega/mavlink.h"

#include "chunk.h"
#include "file.h"

class blockXmit
{
 public:
  blockXmit();
  ~blockXmit();
  bool processMsg(mavlink_message_t &msg);
  void handleAck(mavlink_message_t &msg);
  void handleChunk(mavlink_message_t &msg);
  bool sendChunk(mavlink_message_t &msg);

  void sendFile(const std::string file);
 private:
  std::vector<chunk> qChunk;
  std::map<uint16_t,File> fileMap;
};


#endif
