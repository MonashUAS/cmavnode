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
  File file_(file); //create a file object

  // lock the chunk queue
  std::lock_guard<std::mutex> guard(qChunkMutex);
  file_.createChunks(qChunk); //turn it into chunks on the q
  std::cout << "File added queue size is " << qChunk.size() << std::endl;
}

bool blockXmit::processMsg(mavlink_message_t &msg)
{
  /*
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
  */
}

bool blockXmit::sendChunk(mavlink_message_t &msg)
{
  // lock the chunk queue
  std::lock_guard<std::mutex> guard(qChunkMutex);
  if(qChunk.size() == 0)
    return false;

  //pack the chunk at the front of the vector
  qChunk.at(0).pack(msg);
  std::cout << "Sent Chunk file_id: " << qChunk.at(0).file_id << " chunk_id: " << qChunk.at(0).chunk_id << std::endl;
  std::rotate(qChunk.begin(),qChunk.begin()+1,qChunk.end());

  return true;
}

void blockXmit::handleAck(mavlink_message_t &msg)
{
  uint8_t msg_data[16];
  mavlink_msg_data16_get_data(&msg,msg_data);

  u_split splitter;
  splitter.u8[1] = msg_data[0];
  splitter.u8[0] = msg_data[1];
  uint16_t file_id = splitter.u16;

  splitter.u8[1] = msg_data[2];
  splitter.u8[0] = msg_data[3];
  uint16_t chunk_id = splitter.u16;

  // lock the chunk queue
  std::lock_guard<std::mutex> guard(qChunkMutex);
  std::cout << "Got ACK file_id: " << file_id << " chunk_id: " << chunk_id << std::endl;
  for(auto it = qChunk.begin(); it != qChunk.end(); it++)
  {
    if(it->file_id == file_id && it->chunk_id == chunk_id)
    {
      qChunk.erase(it);
      break;
    }
  }
  /*
  qChunk.erase(std::remove_if(qChunk.begin(),qChunk.end(), [&](chunk const & chunk_)
                              {
                                if(chunk_.file_id == file_id && chunk_.chunk_id == chunk_id)
                                  return true;
                                else
                                  return false;
                              }),
               qChunk.end());
  */
}

void blockXmit::handleChunk(mavlink_message_t &msg, mavlink_message_t &ack)
{
  chunk achunk(msg);

  //see if this is the first chunk for this file
  if(fileMap.count(achunk.file_id) ==0)
  {
    File tmpfile(achunk);
    fileMap.insert(std::make_pair(achunk.file_id,tmpfile));
  }
  else
  { //file already exists
    if(fileMap.at(achunk.file_id).addChunk(achunk))
    { //save file to disk and erase it from the map
      fileMap.at(achunk.file_id).saveFile();
      fileMap.erase(achunk.file_id);
    }
  }

  achunk.genAck(ack);
}

void chunkdiff(chunk c1, chunk c2)
{
  if(memcmp(c1.data,c2.data,sizeof(c1.data)) != 0)
    {
      std::cout << "Found Error in chunk " << c1.chunk_id << ", C1:" << std::endl;
    }
}
