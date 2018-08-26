#ifndef FILE_H
#define FILE_H

#include <iostream>
#include <map>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <iterator>
#include <string>
#include <algorithm>
#include <cmath>

#include "chunk.h"

class File
{
 public:
  File(std::string filename); //create a file by reading from the disk
  File(chunk firstchunk); //create a file because we have received a chunk from it

  void createChunks(std::vector<chunk> &q); //turn a file into chunks

  void addChunk(chunk chunk_); //add a received chunk

  bool isComplete(); //check if all chunks have been received

  void saveFile(); //save a received file to the disk
 private:
  // buffer of bytes that holds the file
  std::vector<char> buffer;
  std::map<uint16_t,bool> rx_map;
  int getFileNumber(std::string filename);

  //number of unique chunks received
  uint16_t rx_count = 0;

  uint16_t filenumber_;
  uint16_t numchunks_;
};

#endif
