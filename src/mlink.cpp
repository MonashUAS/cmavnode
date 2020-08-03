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
std::mutex mlink::sysid_all_links_mutex;
std::set<uint8_t> mlink::sysIDs_all_links;

mlink::mlink(int link_id_, link_options info_)
{
    link_id = link_id_;
    info = info_;
    static_link_delay.push_back(boost::posix_time::time_duration(0,0,0,0));

    drate_thread = boost::thread(&mlink::drateThread, this);
}

int mlink::getLinkID()
{
    return link_id;
}

void mlink::qAddOutgoing(mavlink_message_t msg)
{
    if(!is_kill)
    {
        if(qMavOut.push(msg))
        {
            out_counter.increment();
            totalPacketSent++;
        }
        else
        {
            std::cout << "MLINK: The outgoing queue is full" << std::endl;
        }
    }
}

void mlink::drateThread()
{
    while(!exitFlag)
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(DRATE_PERIOD_MS));
        {
            std::lock_guard<std::mutex> lock(drate_lock_);
            float drate_this_period = (float)drate_rx_bytes_last_period/((float)DRATE_PERIOD_MS/1000.0);
            static const float alpha = 0.2;
            drate_smooth = (alpha * drate_this_period) + (1.0 - alpha) * drate_smooth;
            drate_rx_bytes_last_period = 0;
            std::lock_guard<std::mutex> lock2(stats_lock_);
            link_stats_.drate_rx = drate_smooth;
        }
    }
}

bool mlink::qReadIncoming(mavlink_message_t *msg)
{
    //Will return true if a message was returned by refference
    //false if the incoming queue is empty
    if(qMavIn.pop(*msg))
    {
        in_counter.decrement();
        return true;
    }
    else return false;
}

bool mlink::seenSysID(const uint8_t sysid) const
{
    // returns true if this system ID has been seen on this link
    for (auto iter = sysID_stats.begin(); iter != sysID_stats.end(); ++iter)
    {
        uint8_t this_id = iter->first;
        if (this_id == sysid)
        {
            return true;
        }
    }
    return false;
}

void mlink::onMessageRecv(mavlink_message_t *msg)
{
    //lock up the sysID_stats object
    std::lock_guard<std::mutex> slock_(sysid_lock_);
    updateRouting(*msg);

    record_packet_stats(msg);

    // SiK radio info
    if (info.SiK_radio && (msg->msgid == 109 || msg->msgid == 166))
    {
        //This packet contains info about the radio link. Use the info then discard
        handleSiKRadioPacket(msg);
        return;
    }

    if (record_incoming_packet(msg) == false)
    {
        return;
    }

    //We have made it this far, no reason to drop packet so add to queue
    if(qMavIn.push(*msg))
    {
        in_counter.increment();
    }
    else
    {
        std::cout << "The incoming message queue is full" << std::endl;
    }

    return;
}

void mlink::handleSiKRadioPacket(mavlink_message_t *msg)
{

    std::lock_guard<std::mutex> lock(stats_lock_);
    link_stats_.local_rssi = _MAV_RETURN_uint8_t(msg,  4);
    link_stats_.remote_rssi = _MAV_RETURN_uint8_t(msg,  5);
    link_stats_.tx_buffer = _MAV_RETURN_uint8_t(msg,  6);
    link_stats_.local_noise = _MAV_RETURN_uint8_t(msg,  7);
    link_stats_.remote_noise = _MAV_RETURN_uint8_t(msg,  8);
    link_stats_.rx_errors = _MAV_RETURN_uint16_t(msg,  0);
    link_stats_.corrected_packets = _MAV_RETURN_uint16_t(msg,  2);
}

void mlink::printPacketStats()
{
    std::cout << "PACKET STATS FOR LINK: " << info.link_name << std::endl;

    std::map<uint8_t, sysid_stats>::iterator iter;
    for (iter = sysID_stats.begin(); iter != sysID_stats.end(); ++iter)
    {
        std::cout << "sysID: " << (int)iter->first
                  << " # packets: " << iter->second.num_packets_received
                  << std::endl;
    }
}

void mlink::updateRouting(mavlink_message_t &msg)
{
    bool newSysID;
    auto found = sysID_stats.find(msg.sysid);
    if (found == sysID_stats.end())
    {
        std::cout << "Adding sysID: " << (int)msg.sysid << " to the mapping on link: " << info.link_name << std::endl;
        sysID_stats[msg.sysid].num_packets_received = 0;

        std::lock_guard<std::mutex> lock(sysid_all_links_mutex);
        sysIDs_all_links.insert(sysIDs_all_links.end(), msg.sysid);
        newSysID = true;
        found = sysID_stats.find(msg.sysid);
    }

    struct sysid_stats &stats = found->second;

    stats.num_packets_received++;

    boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();
    stats.last_packet_time = nowTime;

    // Track link delay using heartbeats

    if (msg.msgid == 0 && newSysID == false)
    {
        std::lock_guard<std::mutex> lock(stats_lock_);
        boost::posix_time::time_duration delay = nowTime
                - link_stats_.last_heartbeat
                - boost::posix_time::time_duration(0,0,1,0);
        link_stats_.link_delay = delay.seconds();
        link_stats_.last_heartbeat = nowTime;

        // Remove old packets from recently_received
        std::lock_guard<std::mutex> lock2(recently_received_mutex);
        flush_recently_read();
    }
}

void mlink::checkForDeadSysID()
{
    // lock up sysid_stats object
    std::lock_guard<std::mutex> slock_(sysid_lock_);
    //Check that no links have timed out
    //if they have, remove from mapping

    //get the time now
    boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();

    auto itr = sysID_stats.begin();
    while (itr != sysID_stats.end())
    {
        boost::posix_time::time_duration dur = nowTime - itr->second.last_packet_time;
        long time_between_packets = dur.total_milliseconds();
        if (time_between_packets > MAV_PACKET_TIMEOUT_MS && totalPacketCount > 0)
        {
            std::cout << "Removing sysID: " << (int)(itr->first) << " from link: " << info.link_name << " (idle "  << (double)time_between_packets/1000 << " s)" << std::endl;
            itr = sysID_stats.erase(itr);
        }
        else
        {
            ++itr;
        }
    }
}

bool mlink::record_incoming_packet(mavlink_message_t *msg)
{
    // Returns false if the packet has already been seen and won't be forwarded

    // Extract the mavlink packet into a buffer
    uint8_t snapshot_array[msg->len + 24];
    mavlink_msg_to_send_buffer(snapshot_array, msg);

    // Don't drop heartbeats and only drop when enabled
    if (msg->msgid == 0 || info.reject_repeat_packets == false)
        return true;

    // Ensure link threads don't cause seg faults
    std::lock_guard<std::mutex> lock(recently_received_mutex);

    // Check for repeated packets by comparing checksums
    uint16_t payload_crc;
    if (msg->magic == 254)
    {
        payload_crc = crc_calculate(snapshot_array + 6, msg->len);
    }
    else if (msg->magic == 253)
    {
        payload_crc = crc_calculate(snapshot_array + 11, msg->len);
    }

    // Check whether this packet has been seen before
    if (recently_received[msg->sysid].find(payload_crc) == recently_received[msg->sysid].end())
    {
        // New packet - add it
        boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();
        recently_received[msg->sysid].insert({payload_crc, nowTime});
        return true;
    }
    else
    {
        // Old packet - drop it
        if (sysID_stats.find(msg->sysid) != sysID_stats.end())
        {
            std::cout << "incrementing dropped packets" << std::endl;
            ++sysID_stats[msg->sysid].packets_dropped;
        }
        else
            std::cout << "Failed to find sysid when dropping packet" << std::endl;
        return false;
    }
}

void mlink::flush_recently_read()
{

    std::lock_guard<std::mutex> slock_(sysid_all_links_mutex);
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

void mlink::record_packet_stats(mavlink_message_t *msg)
{

    //increment link packet counter and sysid packet counter
    totalPacketCount++;

    auto found = sysID_stats.find(msg->sysid);
    if (found == sysID_stats.end())
    {
        std::cout << "Failed to find sysid " << (int)msg->sysid << " on link" << info.link_name << " when recording packet stats" << std::endl;
        return;
    }

    struct sysid_stats &stats = found->second;

    stats.recent_packets_received++;

    if (msg->msgid != 109 && msg->msgid != 166 && totalPacketCount > 1)
    {
        if (stats.last_packet_sequence > msg->seq)
        {
            //update total packet loss
            stats.packets_lost += msg->seq
                                  - stats.last_packet_sequence
                                  + 255;
            //update recent packet loss
            stats.recent_packets_lost += msg->seq
                                         - stats.last_packet_sequence
                                         + 255;
        }
        else if (stats.last_packet_sequence < msg->seq)
        {
            //update total packet loss
            stats.packets_lost += msg->seq
                                  - stats.last_packet_sequence
                                  - 1;
            //update recent packet loss
            stats.recent_packets_lost += msg->seq
                                         - stats.last_packet_sequence
                                         - 1;
        }

        //Every 32 packets, use recent packets lost and recent packets received to calculate packet loss percentage
        if((stats.num_packets_received & 0x1F) == 0)
        {
            float packet_loss_percent_ = (float)stats.recent_packets_lost/((float)stats.recent_packets_received + (float)stats.recent_packets_lost);
            packet_loss_percent_ *= 100.0f;
            stats.recent_packets_lost = 0;
            stats.recent_packets_received = 0;
            stats.packet_loss_percent = packet_loss_percent_;

        }
    }

    stats.last_packet_sequence = msg->seq;

    std::lock_guard<std::mutex> lock(drate_lock_);
    drate_rx_bytes_last_period += msg->len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}
