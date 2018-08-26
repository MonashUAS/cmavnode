#include "blockxmit.h"

blockXmit::blockXmit()
{
  std::cout << "Block XMIT initialised" << std::endl;
}

blockXmit::~blockXmit()
{
  std::cout << "Block XMIT destructed" << std::endl;
}

void chunkdiff(chunk c1, chunk c2)
{
  if(memcmp(c1.data,c2.data,sizeof(c1.data)) != 0)
  {
    std::cout << "Found Error in chunk " << c1.chunk_id << ", C1:" << std::endl;
    for(int i = 0; i < BLOCK_XMIT_DATA_BYTES; i++)
    {
      // std::cout << i << ": " << std::setw(3) << std::setfill('0') <<(int)c1.data[i] << " ";
    }
    //std::cout <<std::endl<< "C2" << std::endl;
    for(int i = 0; i < BLOCK_XMIT_DATA_BYTES; i++)
      {
        // std::cout << std::setw(3) << std::setfill('0') <<(int)c2.data[i] << " ";
      }
    //std::cout << std::endl << "end error" << std::endl;
  }
}

void blockXmit::sendFile(const std::string file)
{
  std::cout << "Sending File " << file << std::endl;
  {
    File file_(file); //create a file object
    file_.createChunks(qChunk); //turn it into chunks on the q
  }

  std::cout << "Queue Size is " << qChunk.size() << std::endl;

  //make a msg and pack into it
  mavlink_message_t msg1;
  qChunk.at(0).pack(msg1);


  //make a chunk from decoding the message
  chunk chunk_from_msg(msg1);
  File rxfile_(chunk_from_msg);

  for(int i = 1; i < qChunk.size(); i++)
  {
    mavlink_message_t msg;
    qChunk.at(i).pack(msg);

    chunk chunk_from_msg(msg);
    rxfile_.addChunk(chunk_from_msg);

    chunkdiff(qChunk.at(i),chunk_from_msg);
  }

  if(rxfile_.isComplete())
  {
    std::cout << "File complete" << std::endl;
    rxfile_.saveFile();
  }
  else
    std::cout << "File not complete" << std::endl;

}
