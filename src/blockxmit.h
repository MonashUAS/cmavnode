#ifndef BLOCKXMIT_H
#define BLOCKXMIT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <iterator>
#include <string>
#include "../include/mavlink2/ardupilotmega/mavlink.h"

#define BLOCK_XMIT_DATA_BYTES 60
#define BLOCK_XMIT_SYSID_TX 72
#define BLOCK_XMIT_SYSID_RX 73

union u_split
{
  uint16_t u16;
  uint8_t u8[2];
};

class chunk
{
 public:
  chunk(){};
  chunk(uint16_t file_id, uint16_t chunk_id): file_id(file_id),chunk_id(chunk_id) {};

  uint16_t file_id;
  uint16_t chunk_id;
  uint8_t data[BLOCK_XMIT_DATA_BYTES];

  void pack(mavlink_message_t &msg);
 private:

};

class blockXmit
{
 public:
  blockXmit();
  ~blockXmit();

  void sendFile(const std::string file);
 private:
  std::queue<chunk> qChunk;
};


#endif
