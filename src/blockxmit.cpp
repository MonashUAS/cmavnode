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

  std::ifstream infile(file, std::ios_base::binary | std::ios::ate);
  std::streamsize size = infile.tellg();
  infile.seekg(0,std::ios::beg);
  std::vector<char> buffer(size);
  if(infile.read(buffer.data(),size))
  {
    std::cout << "File read into memory, size: " << buffer.size()/1000 << " kB" << std::endl;
  }
  else
    return;

  int pointer = 0;
  int chunk_id = 0;
  bool fileend = false;
  while(!fileend)
  {
    chunk chunk_(0001,chunk_id++);
    for(int i = 0; i < BLOCK_XMIT_DATA_BYTES; i++)
    {
      if(pointer < buffer.size())
        chunk_.data[i] = buffer.at(pointer++);
      else
      {
        chunk_.data[i] = 0;
        fileend = true;
      }
    }

    std::cout  << "Packed chunk_id " << chunk_id -1 << std::endl;
    qChunk.push(chunk_);
  }

  std::cout << "queue is " << qChunk.size() <<std::endl;

  std::vector<char> buffer_out;
  chunk tmp_chunk;
  while(!qChunk.empty())
  {
    tmp_chunk = qChunk.front();
    qChunk.pop();
    for(int i = 0; i < BLOCK_XMIT_DATA_BYTES; i++)
    {
      buffer_out.push_back(tmp_chunk.data[i]);
    }
  }
  std::ofstream outfile("test.jpg", std::ios::out | std::ios::binary);
  outfile.write(buffer_out.data(), buffer_out.size());
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

  for(int i = 0; i < BLOCK_XMIT_DATA_BYTES; i++)
  {
    buffer[4+i] = data[i];
  }
  //now we have our 64 bytes ready

  mavlink_msg_data64_pack(BLOCK_XMIT_SYSID_TX,1,&msg,1,64,buffer);
}
