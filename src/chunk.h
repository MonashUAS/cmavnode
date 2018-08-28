#ifndef CHUNK_H
#define CHUNK_H

#include <iostream>
#include "../include/mavlink2/ardupilotmega/mavlink.h"

#define BLOCK_XMIT_DATA_BYTES 57
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
  //Create a chunk on tx size from the known data
 chunk(uint16_t file_id, uint16_t chunk_id,uint16_t num_chunks): file_id(file_id),chunk_id(chunk_id),num_chunks(num_chunks) {};

  //Create a chunk on the rx side from a mavlink message
  chunk(mavlink_message_t &msg);

  void printData();
  void genAck(mavlink_message_t &msg);

  uint16_t file_id;
  uint16_t chunk_id;
  uint16_t num_chunks;
  uint8_t data[BLOCK_XMIT_DATA_BYTES];

  void pack(mavlink_message_t &msg);
 private:

};


#endif
