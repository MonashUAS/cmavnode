#include "chunk.h"

chunk::chunk(mavlink_message_t &msg)
{
  //get data from msg into a array
  uint8_t msg_data[64];
  mavlink_msg_data64_get_data(&msg,msg_data);

  u_split splitter;
  splitter.u8[1] = msg_data[0];
  splitter.u8[0] = msg_data[1];
  file_id = splitter.u16;

  splitter.u8[1] = msg_data[2];
  splitter.u8[0] = msg_data[3];
  chunk_id = splitter.u16;

  splitter.u8[1] = msg_data[4];
  splitter.u8[0] = msg_data[5];
  num_chunks = splitter.u16;

  for(int i = 0; i < BLOCK_XMIT_DATA_BYTES; i++)
  {
    data[i] = msg_data[6+i];
  }
}

void chunk::pack(mavlink_message_t &msg)
{
  uint8_t buffer[64];
  u_split splitter;
  splitter.u16 = file_id;
  buffer[0] = splitter.u8[1];
  buffer[1] = splitter.u8[0];

  splitter.u16 = chunk_id;
  buffer[2] = splitter.u8[1];
  buffer[3] = splitter.u8[0];

  splitter.u16 = num_chunks;
  buffer[4] = splitter.u8[1];
  buffer[5] = splitter.u8[0];

  for(int i = 0; i < BLOCK_XMIT_DATA_BYTES; i++)
    {
      buffer[6+i] = data[i];
    }

  buffer[63] = 0xff;
  //now we have our 64 bytes ready

  mavlink_msg_data64_pack(BLOCK_XMIT_SYSID_TX,1,&msg,1,64,buffer);
}

void chunk::genAck(mavlink_message_t &msg)
{
  uint8_t buffer[4];

  u_split splitter;
  splitter.u16 = file_id;
  buffer[0] = splitter.u8[1];
  buffer[1] = splitter.u8[0];

  splitter.u16 = chunk_id;
  buffer[2] = splitter.u8[1];
  buffer[3] = splitter.u8[0];

  //Now we have a complete ack buffer
  mavlink_msg_data16_pack(BLOCK_XMIT_SYSID_RX,1,&msg,1,4,buffer);
}

void chunk::printData()
{
  for(int i = 0; i < BLOCK_XMIT_DATA_BYTES; i ++)
    {
      std::cout << i << ": " <<(int)data[i] << " ";
    }
  std::cout << std::endl;
}
