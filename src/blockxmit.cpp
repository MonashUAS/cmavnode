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

  std::cout << "Queue Size is " << qChunk.size() << std::endl;

  File rxfile_(qChunk.at(0));
  for(int i = 1; i < qChunk.size(); i++)
  {
    rxfile_.addChunk(qChunk.at(i));
  }

  if(rxfile_.isComplete())
  {
    std::cout << "File complete" << std::endl;
    rxfile_.saveFile();
  }
  else
    std::cout << "File not complete" << std::endl;

}
