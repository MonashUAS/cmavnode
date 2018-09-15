#include "file.h"

File::File(std::string filename,uint16_t x, uint16_t y)
{
  filename_ = filename;
  std::ifstream infile(filename, std::ios_base::binary | std::ios::ate);
  std::streamsize size = infile.tellg();
  buffer.resize(size);
  infile.seekg(0,std::ios::beg);
  if(infile.read(buffer.data(),size))
    {
      std::cout << "File read into memory, size: " << buffer.size() << " B" << std::endl;
    }
  else
    std::cout << "Cannot read file FAIL" << std::endl;

  // add the x y info to the front of the buffer
  u_split splitter;
  splitter.u16 = x;
  buffer.insert(buffer.begin(),splitter.u8[0]);
  buffer.insert(buffer.begin(),splitter.u8[1]);
  splitter.u16 = y;
  buffer.insert(buffer.begin(),splitter.u8[0]);
  buffer.insert(buffer.begin(),splitter.u8[1]);

  // populate info fields
  filenumber_ = getFileNumber(filename);
  numchunks_ = ceil((float)size/(float)BLOCK_XMIT_DATA_BYTES);
}

File::File(chunk firstchunk)
{
  // save info from the chunk
  filenumber_ = firstchunk.file_id;
  filenumberstring_ = std::to_string(filenumber_);
  filenumberstring_ = std::string(4 - filenumberstring_.length(), '0') + filenumberstring_;
  filename_ = filenumberstring_ + ".jpg";

  numchunks_ = firstchunk.num_chunks;
  first_ts = boost::posix_time::microsec_clock::local_time();

  //preallocate the rx buffer
  buffer.resize(numchunks_*BLOCK_XMIT_DATA_BYTES);

  std::cout << "First chunk registered, buffer created as " << buffer.size() << std::endl;

  //preallocate the rx map indicating all chunks not received
  for(int i = 0; i < numchunks_; i++)
  {
    rx_map[i] = false;
  }

  // now call the generic chunk adder
  addChunk(firstchunk);
}

bool File::isComplete()
{
  return rx_count == numchunks_;
}

void File::createChunks(std::vector<chunk> &q)
{
  int pointer = 0;
  int chunk_id = 0;
  bool fileend = false;
  while(!fileend)
    {
      chunk chunk_(filenumber_,chunk_id++,numchunks_);
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
      q.push_back(chunk_);
    }
}

void File::printTransferTime()
{
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = now - first_ts;
    long ms = diff.total_milliseconds();
    if(ms < 10000)
      std::cout << "File " << filenumber_ << " fully received in " << ms << " milliseconds" << std::endl;
    else
      std::cout << "File " << filenumber_ << " fully received in " << diff.total_seconds() << " seconds" << std::endl;
}

bool File::addChunk(chunk chunk_)
{
  //do nothing if we have this chunk already
  if(rx_map[chunk_.chunk_id] == true)
    return isComplete();

  // record that we have this chunk
  rx_map[chunk_.chunk_id] = true;
  rx_count++;

  //write it into the buffer
  for(int i = 0; i < BLOCK_XMIT_DATA_BYTES; i++)
  {
    buffer[chunk_.chunk_id*BLOCK_XMIT_DATA_BYTES + i] = chunk_.data[i];
  }

  // std::cout << "Added chunk file id: "
  //           << chunk_.file_id << " chunk id: "
  //           << chunk_.chunk_id << " num chunks: "
  //           << chunk_.num_chunks << std::endl;

  return isComplete();
}

void File::saveFile()
{

  // Strip x and y from front of buffer
  u_split splitter;
  splitter.u8[1] = buffer.front();
  buffer.erase(buffer.begin());
  splitter.u8[0] = buffer.front();
  buffer.erase(buffer.begin());
  int y = splitter.u16;
  splitter.u8[1] = buffer.front();
  buffer.erase(buffer.begin());
  splitter.u8[0] = buffer.front();
  buffer.erase(buffer.begin());
  int x = splitter.u16;

  std::ofstream myfile;
  myfile.open ("cmavoutput/" +filenumberstring_+".txt");
  myfile << "x: " << x << "\n";
  myfile << "y: " << y << "\n";
  myfile.close();

  std::ofstream outfile("cmavoutput/" + filename_, std::ios::out | std::ios::binary);
  outfile.write(buffer.data(), buffer.size());
}

int File::getFileNumber(std::string filename)
{
  std::string base = filename.substr(filename.find_last_of("/") +1);
  std::string::size_type const p(base.find_last_of('.'));
  std::string base_no_ext = base.substr(0,p);
  std::stringstream b_stream(base_no_ext);
  int filenumber = 0;
  b_stream >> filenumber;
  return filenumber;
}
