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
#include <mutex>
#include <boost/format.hpp>
#include <string>
#include "../include/mavlink2/ardupilotmega/mavlink.h"

#include "chunk.h"
#include "file.h"

#define BLOCK_XMIT_RUN_EVERY_MS 100
#define BLOCK_XMIT_CHUNKS_PER_RUN 2

class blockXmit
{
 public:
  blockXmit();
  ~blockXmit();
  bool processMsg(mavlink_message_t &msg);
  void handleAck(mavlink_message_t &msg);
  void handleChunk(mavlink_message_t &msg,mavlink_message_t &ack);
  bool sendChunk(mavlink_message_t &msg);
  void sendFile(const std::string file);

 private:
  //the chunk queue will be accessed from the web server and the main loop, so we need to protect it
  std::mutex qChunkMutex;
  std::vector<chunk> qChunk;

  //file map only ever accessed from main loop on rx
  std::map<uint16_t,File> fileMap;

  std::vector<uint16_t> completedFileMap;
};


#endif
