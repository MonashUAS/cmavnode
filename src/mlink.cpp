/* CMAVNode
 * Monash UAS
 *
 * LINK CLASS
 * This is a virtual class which will be extended by various types of links
 * All methods which need to be accessed from the main thread
 * need to be declared here and overwritten
 */

#include "mlink.h"

std::unordered_map<uint8_t, std::map<uint16_t, boost::posix_time::ptime> > mlink::recently_received;
std::vector<boost::posix_time::time_duration> mlink::static_link_delay;
std::mutex mlink::recently_received_mutex;
std::set<uint8_t> mlink::sysIDs_all_links;

mlink::mlink(link_info info_)
{
    info = info_;
    static_link_delay.push_back(boost::posix_time::time_duration(0,0,0,0));

    //if we are simulating init the random generator
    if( info.sim_enable) srand(time(NULL));
}

void mlink::qAddOutgoing(mavlink_message_t msg)
{
    if(!is_kill)
    {
        bool returnCheck = qMavOut.push(msg);
        recentPacketSent++;

        if(!returnCheck) //Then the queue is full
        {
            LOG(ERROR) << "MLINK: The outgoing queue is full";
        }
    }
}

bool mlink::qReadIncoming(mavlink_message_t *msg)
{
    //Will return true if a message was returned by refference
    //false if the incoming queue is empty
    return qMavIn.pop(*msg);
}

void mlink::getSysID_thisLink()
{

}

bool mlink::onMessageRecv(mavlink_message_t *msg)
{
    recentPacketCount++;
    // If the message was a heartbeat, update (or add) that system ID
    if(msg->msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        onHeartbeatRecv(msg->sysid);
    }
    else if (msg->msgid == 109)  // If the message is about the link, update the link stats
    {
      // Update link quality stats for this link
      link_quality.local_rssi = _MAV_RETURN_uint8_t(msg,  4);
      link_quality.remote_rssi = _MAV_RETURN_uint8_t(msg,  5);
      link_quality.tx_buffer = _MAV_RETURN_uint8_t(msg,  6);
      link_quality.local_noise = _MAV_RETURN_uint8_t(msg,  7);
      link_quality.remote_noise = _MAV_RETURN_uint8_t(msg,  8);
      link_quality.rx_errors = _MAV_RETURN_uint16_t(msg,  0);
      link_quality.corrected_packets = _MAV_RETURN_uint16_t(msg,  2);
    }
    return true;
}

bool mlink::shouldDropPacket()
{
    if(info.sim_enable)
    {
        int randnumber = rand() % 100 + 1;
        if(randnumber < info.sim_packet_loss)
        {
            return true;
        }
    }
    return false;
}

void mlink::printHeartbeatStats()
{
    std::cout << "HEARTBEAT STATS FOR LINK: " << info.link_name << std::endl;

    std::map<uint8_t, heartbeat_stats>::iterator iter;
    for (iter = sysID_stats.begin(); iter != sysID_stats.end(); ++iter)
    {
        std::cout << "sysID: " << (int)iter->first
                  << " # heartbeats: " << iter->second.num_heartbeats_received
                  << std::endl;
    }
}

void mlink::onHeartbeatRecv(uint8_t sysID)
{
    // Search for the given system ID
    std::map<uint8_t, heartbeat_stats>::iterator iter;
    std::pair<std::map<uint8_t, heartbeat_stats>::iterator, bool> ret;
    // If the system ID is new, add it to the map and set.
    // Return the position of the new or existing element
    ret = sysID_stats.insert(std::pair<uint8_t,heartbeat_stats>(sysID,heartbeat_stats()));
    sysIDs_all_links.insert(sysIDs_all_links.end(), sysID);

    iter = ret.first;

    // Record when the heartbeat was received
    boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();

    // Update the map for this system ID
    iter->second.num_heartbeats_received++;
    iter->second.last_heartbeat_time = nowTime;

    // Also update the link delay
    boost::posix_time::time_duration delay = nowTime
                                                - link_quality.last_heartbeat
                                                - boost::posix_time::time_duration(0,0,1,0);
    // Don't allow negative delay
    if (delay < boost::posix_time::time_duration(0,0,0,0))
        delay = boost::posix_time::time_duration(0,0,0,0);
    link_quality.link_delay = delay;
    link_quality.last_heartbeat = nowTime;

    // Check whether the system ID is new and log if it is
    if (ret.second == true)
        LOG(INFO) << "Adding sysID: " << (int)sysID << " to the mapping on link: " << info.link_name;

    // Clear out old sysIDs
    checkForDeadSysID();

    // Remove old packets from recently_received
    std::lock_guard<std::mutex> lock(recently_received_mutex);
    flush_recently_read();
}


void mlink::checkForDeadSysID()
{
    //Check that no links have timed out
    //if they have, remove from mapping

    //get the time now
    boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();

    // Iterating through the map
    std::map<uint8_t, heartbeat_stats>::iterator iter;
    for (iter = sysID_stats.begin(); iter != sysID_stats.end(); ++iter)
    {
      boost::posix_time::time_duration dur = nowTime - iter->second.last_heartbeat_time;
      long time_between_heartbeats = dur.total_milliseconds();

      if(time_between_heartbeats > MAV_HEARTBEAT_TIMEOUT_MS)  // Check for timeout
      {
        // Clarify why links drop out due to timing out
        LOG(INFO) << "sysID: " << (int)(iter->first) << " timed out after " << (double)time_between_heartbeats/1000 << " s.";
        // Log then erase
        LOG(INFO) << "Removing sysID: " << (int)(iter->first) << " from the mapping on link: " << info.link_name;
        sysID_stats.erase(iter);
      }
    }
}


bool mlink::record_incoming_packet(mavlink_message_t &msg)
{
    // Check incoming bytes for parts of a mavlink packet
    // See http://qgroundcontrol.org/mavlink/start for mavlink packet anatomy
    // Returns false if the packet has already been seen and won't be recorded

    // Note that custom messages have had their crcs found by brute force
    static uint8_t mavlink_message_crcs[256] = { 50, 124, 137,   0, 237, 217, 104, 119,
                                                  0,   0,   0,  89,   0,   0,   0,   0,
                                                  0,   0,   0,   0, 214, 159, 220, 168,
                                                 24,  23, 170, 144,  67, 115,  39, 246,
                                                185, 104, 237, 244, 222, 212,   9, 254,
                                                230,  28,  28, 132, 221, 232,  11, 153,
                                                 41,  39,  78, 196,   0,   0,  15,   3,
                                                  0,   0,   0,   0,   0, 167, 183, 119,
                                                191, 118, 148,  21,   0, 243, 124,   0,
                                                  0,  38,  20, 158, 152, 143,   0,   0,
                                                  0, 106,  49,  22, 143, 140,   5, 150,
                                                  0, 231, 183,  63,  54,  47,   0,   0,
                                                  0,   0,   0,   0, 175, 102, 158, 208,
                                                 56,  93, 138, 108,  32, 185,  84,  34,
                                                174, 124, 237,   4,  76, 128,  56, 116,
                                                134, 237, 203, 250,  87, 203, 220,  25,
                                                226,  46,  29, 223,  85,   6, 229, 203,
                                                  1, 195, 109, 168, 181,  47,  72, 131,
                                                127,   0, 103, 154, 178, 200, 134,   0,
                                                208,   0,   0,   0,   0,   0,   0,   0,
                                                  0,   0,   0, 127, 154,  21,  22,   0,
                                                  1,   0,   0,   0,   0,   0, 167,   0,
                                                  0,   0,  47,   0,   0,   0, 229,   0,
                                                  0,   0,   0,   0,   0,   0,   0,   0,
                                                  0,  71,   0,   0,   0,   0,   0,   0,
                                                  0,   0,   0,   0,   0,   0,   0,   0,
                                                  0,   0,   0,   0,   0,   0,   0,   0,
                                                  0,   0,   0,   0,   0,   0,   0,   0,
                                                  0,   0,   0,   0,   0,   0, 163, 105,
                                                151,  35, 150,   0,   0,   0,   0,   0,
                                                  0,  90, 104,  85,  95, 130, 184,  81,
                                                  8, 204,  49, 170,  44,  83,  46,   0};

    // Extract the mavlink packet into a buffer
    static uint8_t snapshot_array[263];
    snapshot_array[0] = 254;
    snapshot_array[1] = msg.len;
    snapshot_array[2] = msg.seq;
    snapshot_array[3] = msg.sysid;
    snapshot_array[4] = msg.compid;
    snapshot_array[5] = msg.msgid;
    _MAV_RETURN_uint8_t_array(&msg, snapshot_array + 6, msg.len, 0);

    // Track packets loss
    // Deal with wrapping of 8 bit integer
    if (msg.msgid != 109 && msg.msgid != 166)
    {
        // Ignore packet sequences from RFDs
        if (link_quality.last_packet_sequence > msg.seq)
            link_quality.packets_lost += msg.seq
                                        - link_quality.last_packet_sequence
                                        + 255;
        else
            link_quality.packets_lost += msg.seq
                                        - link_quality.last_packet_sequence
                                        - 1;
        link_quality.last_packet_sequence = msg.seq;
    }

    if (149 < msg.msgid && msg.msgid < 230
        && mavlink_message_crcs[msg.msgid] == 0
        && new_custom_msg_crcs.find(msg.msgid) == new_custom_msg_crcs.end())
    {
        // These custom messages have not been encountered before and need their
        // crcs to be added to mavlink_message_crcs

        // Brute-force to find the correct crc
        uint8_t crc_extra_guess = 1;
        uint16_t tryChecksum = 0;
        while (msg.checksum != tryChecksum)
        {
            tryChecksum = crc_calculate(snapshot_array + 1, msg.len + 5);
            crc_accumulate(crc_extra_guess++, &tryChecksum);
        }
        mavlink_message_crcs[msg.msgid] = crc_extra_guess;

        LOG(ERROR) << "Custom mavlink packet with msgID " << (int)msg.msgid
                   << " detected. This packet did not have a known crc"
                   << " extra byte, however it has been calculated to be "
                   << (int)crc_extra_guess << ". Please add this value to the"
                   << " \"mavlink_message_crcs\" array to avoid this error"
                   << " in future.";
        new_custom_msg_crcs.insert(std::make_pair(msg.msgid, crc_extra_guess));
    }

    // Use the incoming packet to calculate two new checksum bytes for if/when
    // it is forwarded with a new sequence number
    msg.seq = ++link_quality.out_packet_sequence;
    snapshot_array[2] = link_quality.out_packet_sequence;
    uint16_t checksum = crc_calculate(snapshot_array + 1, msg.len + 5);
    crc_accumulate(mavlink_message_crcs[msg.msgid], &checksum); // crc extra
    msg.checksum = checksum;

    // Ensure link threads don't cause seg faults
    std::lock_guard<std::mutex> lock(recently_received_mutex);

    // Don't drop heartbeats and only drop when enabled
    if (msg.msgid == 0 || info.reject_repeat_packets == false)
        return true;

    // Check for repeated packets by comparing checksums
    uint16_t payload_crc = crc_calculate(snapshot_array + 6, msg.len);

    // Check whether this packet has been seen before
    if (recently_received[msg.sysid].find(payload_crc) == recently_received[msg.sysid].end())
    {
        // New packet - add it
        boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();
        recently_received[msg.sysid].insert({payload_crc, nowTime});
        return true;
    } else
    {
        // Old packet - drop it
        ++link_quality.packets_dropped;
        return false;
    }
}

void mlink::flush_recently_read()
{
    for (auto sysID = sysIDs_all_links.begin(); sysID != sysIDs_all_links.end(); ++sysID)
    {
        auto recent_packet_map = &recently_received[*sysID];
        for (auto packet = recent_packet_map->begin(); packet != recent_packet_map->end(); ++packet)
        {
            boost::posix_time::time_duration elapsed_time = boost::posix_time::microsec_clock::local_time() - packet->second;
            if (elapsed_time > boost::posix_time::time_duration(0,0,1,0) &&
                elapsed_time > max_delay())
            {
                recent_packet_map->erase(packet);
            }
        }
    }
}

boost::posix_time::time_duration mlink::max_delay()
{
    boost::posix_time::time_duration ret = boost::posix_time::time_duration(0,0,0,0);
    for (auto delay = static_link_delay.begin(); delay != static_link_delay.end(); ++delay)
    {
        if ((*delay) > ret)
            ret = *delay;
    }
    return ret;
}
