/* CMAVNode
 * Monash UAS
 *
 * LINK CLASS
 * This is a virtual class which will be extended by various types of links
 * All methods which need to be accessed from the main thread
 * need to be declared here and overwritten
 */
#ifndef MLINK_H
#define MLINK_H

#include <vector>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include "../include/mavlink/ardupilotmega/mavlink.h"
#include "../include/logging/src/easylogging++.h"
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <map>
#include <vector>
#include <utility>
#include <algorithm>

#include "exception.h"

#define MAV_INCOMING_LENGTH 1000
#define MAV_OUTGOING_LENGTH 1000
#define OUT_QUEUE_EMPTY_SLEEP 50
#define MAV_INCOMING_BUFFER_LENGTH 2041
#define MAV_HEARTBEAT_TIMEOUT_MS 10000

struct link_info{
    std::string link_name;
    int receive_from, output_to, output_only_heartbeat_from;
    std::vector<int> output_only_from;
};

class mlink
{
public:
    mlink(link_info info_);
    virtual ~mlink() {};

    int link_id;

    bool up = true;

    //Send or read mavlink messages
    void qAddOutgoing(mavlink_message_t msg);
    bool qReadIncoming(mavlink_message_t *msg);

    void printHeartbeatStats();

    //update the public mapping based on private one
    void getSysID_thisLink();

    //remove dead systems from private mapping
    void checkForDeadSysID();


    void onHeartbeatRecv(uint8_t sysID);
    void onMessageRecv(mavlink_message_t *msg);

#ifdef MUASMAV
    void hackSysID(mavlink_message_t *msg);
#endif

    //Public system ID mapping
    std::vector<uint8_t> sysIDpub;

    //Read and write thread functions. Read thread will call ioservice.run and block
    //Write thread will be in an infinate busy wait loop
    virtual void runWriteThread() {};
    virtual void runReadThread() {};

    link_info info;

    bool is_kill = false;
    long recentPacketCount = 0;
    long recentPacketSent = 0;

    struct heartbeat_stats {
      int num_heartbeats_received = 0;
      boost::posix_time::ptime last_heartbeat_time;
    };
    // Track heartbeat stats for each system ID.
    std::map<uint8_t, heartbeat_stats> sysID_stats;

    // Track link quality for the link
    struct link_quality_stats {
      int local_rssi = 0;
      int remote_rssi = 0;
      int tx_buffer = 0;
      int local_noise = 0;
      int remote_noise = 0;
      int rx_errors = 0;
      int corrected_packets = 0;
      boost::posix_time::ptime last_heartbeat;
      boost::posix_time::time_duration link_delay;
      int packets_lost = 0;
    };
    link_quality_stats link_quality;

    // Accessor function for recently_read.
    bool record_incoming_packet();

    boost::posix_time::time_duration max_delay();
    void flush_recently_read();

protected:
    boost::lockfree::spsc_queue<mavlink_message_t> qMavIn {MAV_INCOMING_LENGTH};
    boost::lockfree::spsc_queue<mavlink_message_t> qMavOut {MAV_OUTGOING_LENGTH};

    boost::thread read_thread;
    boost::thread write_thread;

    bool exitFlag = false;

    std::vector<uint8_t> data_in_snapshot{std::vector<uint8_t>(263, 0)};
    uint8_t data_in_[MAV_INCOMING_BUFFER_LENGTH];
    uint8_t data_out_[MAV_INCOMING_BUFFER_LENGTH];

    // A record of recent incoming packets is kept to avoid repeated packets
    // over various links to the same system ID.
    // Each sysID is the key to a map of the bytes received in a packet
    static std::unordered_map<uint8_t, std::map<std::vector<uint8_t>, boost::posix_time::ptime> > recently_received;

    // All links have their delay tracked to periodically flush recently_received
    static std::vector<boost::posix_time::time_duration> static_link_delay;
};

#endif
