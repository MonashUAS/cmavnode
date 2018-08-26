#include "chunk.h"

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
  //now we have our 64 bytes ready

  mavlink_msg_data64_pack(BLOCK_XMIT_SYSID_TX,1,&msg,1,64,buffer);
}
