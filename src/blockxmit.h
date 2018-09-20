#ifndef BLOCKXMIT_H
#define BLOCKXMIT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <iterator>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include "../include/mavlink2/ardupilotmega/mavlink.h"

#include "chunk.h"
#include "file.h"

#define BLOCK_XMIT_RUN_EVERY_MS 100 // How often block xmit will transmit on the main loop
#define BLOCK_XMIT_CHUNKS_PER_RUN 1 // Send x chunks on each configured link every run
#define BLOCK_XMIT_MAX_Q 200 // Soft limit on chunks in queue

class blockXmit
{
public:
    blockXmit(std::string rx_dir);
    ~blockXmit();
    bool processMsg(mavlink_message_t &msg);
    void handleAck(mavlink_message_t &msg);
    void handleChunk(mavlink_message_t &msg,mavlink_message_t &ack);
    bool sendChunk(mavlink_message_t &msg);
    bool sendFile(const std::string file,uint16_t x, uint16_t y);

private:
    //the chunk queue will be accessed from the web server and the main loop, so we need to protect it
    std::mutex qChunkMutex;
    std::vector<chunk> qChunk;

    //file map only ever accessed from main loop on rx
    std::map<uint16_t,File> fileMap;

    std::vector<uint16_t> completedFileMap;

    std::string rx_dir_;
};


#endif
