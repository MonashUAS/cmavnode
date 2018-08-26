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

  void sendFile(const std::string file);
 private:
  std::vector<chunk> qChunk;
};


#endif
