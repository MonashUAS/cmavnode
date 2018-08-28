#include "blockxmit.h"

blockXmit::blockXmit()
{
  std::cout << "Block XMIT initialised" << std::endl;
}

blockXmit::~blockXmit()
{
  std::cout << "Block XMIT destructed" << std::endl;
}

void blockXmit::sendFile(const std::string file)
{
  std::cout << "Sending File " << file << std::endl;
  {
    File file_(file); //create a file object
    file_.createChunks(qChunk); //turn it into chunks on the q
  }
  std::cout << "File added queue size is " << qChunk.size() << std::endl;
}

bool blockXmit::processMsg(mavlink_message_t &msg)
{
  if(msg.msgid == MAVLINK_MSG_ID_DATA16 && msg.sysid == BLOCK_XMIT_SYSID_RX)
  {
    handleAck(msg);
  }
  else if(msg.msgid == MAVLINK_MSG_ID_DATA64 && msg.sysid == BLOCK_XMIT_SYSID_TX)
  {
    handleChunk(msg);
  }
  else
    return false;
}

void blockXmit::handleAck(mavlink_message_t &msg)
{
  uint8_t msg_data[4];
  mavlink_msg_data16_get_data(&msg,msg_data);

  u_split splitter;
  splitter.u8[1] = msg_data[0];
  splitter.u8[0] = msg_data[1];
  uint16_t file_id = splitter.u16;

  splitter.u8[1] = msg_data[2];
  splitter.u8[0] = msg_data[3];
  uint16_t chunk_id = splitter.u16;

  qChunk.erase(std::remove_if(qChunk.begin(),qChunk.end(), [&](chunk const & chunk_)
                              {
                                if(chunk_.file_id == file_id && chunk_.chunk_id == chunk_id)
                                  return true;
                                else
                                  return false;
                              }),
               qChunk.end());

}

void blockXmit::handleChunk(mavlink_message_t &msg)
{
  
}

void chunkdiff(chunk c1, chunk c2)
{
  if(memcmp(c1.data,c2.data,sizeof(c1.data)) != 0)
    {
      std::cout << "Found Error in chunk " << c1.chunk_id << ", C1:" << std::endl;
    }
}
