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
#include <atomic>
#include <boost/asio.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include "../include/mavlink2/ardupilotmega/mavlink.h"
#include "../include/mavlink2/checksum.h"
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <map>
#include <vector>
#include <utility>
#include <algorithm>
#include <mutex>
#include <set>

#include "exception.h"

#define MAV_INCOMING_LENGTH 2000
#define MAV_OUTGOING_LENGTH 2000
#define OUT_QUEUE_EMPTY_SLEEP 10
#define MAV_INCOMING_BUFFER_LENGTH 2041
#define MAV_PACKET_TIMEOUT_MS 10000

struct queue_counter
{
    std::atomic<int> value{0};

    void increment()
    {
        ++value;
    }

    void decrement()
    {
        --value;
    }

    int get()
    {
        return value.load();
    }
};

struct link_options
{
    // this struct represents link settings that can change at runtime
    // these settings work at the mlink layer
    std::string link_name;
    bool reject_repeat_packets = false;
    bool SiK_radio = false;
    bool blockXmitRx = false;
    bool blockXmitTx = false;
};

// Stats for the entire link, not specific to system
struct link_stats
{
    int local_rssi = 0;
    int remote_rssi = 0;
    int tx_buffer = 0;
    int local_noise = 0;
    int remote_noise = 0;
    int rx_errors = 0;
    int corrected_packets = 0;
    boost::posix_time::ptime last_heartbeat = boost::posix_time::microsec_clock::local_time();
    long link_delay = 0;
  float drate_rx = 0;
};

// track metrics for one system on one link
struct sysid_stats
{
  int num_packets_received = 0;
  int recent_packets_received = 0;
  int recent_packets_lost = 0;
  boost::posix_time::ptime last_packet_time;
  uint8_t last_packet_sequence = -1;
  uint8_t out_packet_sequence = 0;
  int packets_lost = 0;
  int packets_dropped = 0;
  float packet_loss_percent = 0;
};

struct MlinkCached
{
    virtual ~MlinkCached() {}
    int link_id_;
    link_options link_options_;
  link_stats stats_;
  std::map<uint8_t, sysid_stats> sysid_stats_;
};

class mlink
{
public:
    mlink(int link_id_, link_options info_);
    virtual ~mlink() {};

    bool up = true;

    int getLinkID();

    //Send or read mavlink messages
    void qAddOutgoing(mavlink_message_t msg);
    bool qReadIncoming(mavlink_message_t *msg);

    void printPacketStats();

    // indicate if a system has been seen on a link:
    bool seenSysID(uint8_t sysid) const;

    //remove dead systems from private mapping
    void checkForDeadSysID();


    void updateRouting(mavlink_message_t &msg);
    void onMessageRecv(mavlink_message_t *msg); // returns whether to throw out this message

    //Read and write thread functions. Read thread will call ioservice.run and block
    //Write thread will be in an infinate busy wait loop
    virtual void runWriteThread() {};
    virtual void runReadThread() {};

    link_options info;

    queue_counter out_counter;
    queue_counter in_counter;

    bool is_kill = false;
    long totalPacketCount = 0;
    long totalPacketSent = 0;

    // Datarates are in kB/s and are based on the last 10s of traffic


    std::mutex drate_lock_;
    static const int drate_period_ms = 200;
    float datarate_rx = 0;
    float drate_smooth;
    long drate_rx_bytes_last_period;

    // Track link quality for the link
    std::mutex stats_lock_;
    link_stats link_stats_;


    // Track heartbeat stats for each system ID.
    std::map<uint8_t, sysid_stats> sysID_stats;

    // return endpoint corresponding to sender (if any)
    virtual boost::asio::ip::udp::endpoint *sender_endpoint()
    {
        return nullptr;
    }

protected:
    boost::lockfree::spsc_queue<mavlink_message_t> qMavIn {MAV_INCOMING_LENGTH};
    boost::lockfree::spsc_queue<mavlink_message_t> qMavOut {MAV_OUTGOING_LENGTH};

    boost::thread read_thread;
    boost::thread write_thread;
    boost::thread drate_thread;

    int link_id;

    bool exitFlag = false;

    uint8_t data_in_[MAV_INCOMING_BUFFER_LENGTH];
    uint8_t data_out_[MAV_INCOMING_BUFFER_LENGTH];

    // A record of recent incoming packets is kept to avoid repeated packets
    // over various links to the same system ID.
    // Each sysID is the key to a map of the bytes received in a packet
    static std::unordered_map<uint8_t, std::map<uint16_t, boost::posix_time::ptime> > recently_received;
    static std::mutex recently_received_mutex;

    // Accessor function for recently_read also performs resequencing
    bool record_incoming_packet(mavlink_message_t *msg);
    // Helper functions for record_incoming_packet()
    boost::posix_time::time_duration max_delay();
    void flush_recently_read();
    void record_packet_stats(mavlink_message_t *msg);
    void handleSiKRadioPacket(mavlink_message_t *msg);

    void drateThread();

    // All links have their delay tracked to periodically flush recently_received
    static std::vector<boost::posix_time::time_duration> static_link_delay;

    std::map<uint8_t, uint8_t> new_custom_msg_crcs;

    static std::set<uint8_t> sysIDs_all_links;
};

#endif
