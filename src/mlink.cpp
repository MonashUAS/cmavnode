/* CMAVNode
 * Monash UAS
 *
 * LINK CLASS
 * This is a virtual class which will be extended by various types of links
 * All methods which need to be accessed from the main thread
 * need to be declared here and overwritten
 */

#include "mlink.h"
mlink::mlink(link_info info_)
{
    info = info_;
}

void mlink::qAddOutgoing(mavlink_message_t msg)
{
    if(!is_kill){
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
    //iterate through internal mapping and return sysID's
    checkForDeadSysID();
    // Empty the vector of system IDs
    sysIDpub.clear();

    // Iterate through the system ID stats map and put all of the sys IDs into
    // sysIDpub
    std::map<uint8_t, heartbeat_stats>::iterator iter;
    for (iter = sysID_stats.begin(); iter != sysID_stats.end(); ++iter)
    {
      sysIDpub.push_back(iter->first);
    }
}

void mlink::onMessageRecv(mavlink_message_t *msg)
{
    //Check if this message needs special handling based on content

    recentPacketCount++;
    // If the message was a heartbeat, update (or add) that system ID
    if(msg->msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        onHeartbeatRecv(msg->sysid);
    }
    else if (msg->msgid == 166)  // If the message is about the link, update the link stats
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
    else if (msg->msgid == 179) // If the message was a GPS timestamp, update the link stats
    {
      // Time when packet was sent
      boost::posix_time::time_duration mins_remote =  boost::posix_time::minutes(_MAV_RETURN_uint8_t(msg,  4));
      boost::posix_time::time_duration secs_remote = boost::posix_time::seconds(_MAV_RETURN_uint8_t(msg,  5));
      // Current time
      boost::posix_time::ptime time_local = boost::posix_time::second_clock::universal_time();
      //Calculate packet delay
      link_quality.link_delay = to_simple_string(time_local - mins_remote
                                                - secs_remote).substr(15,5);
    }
}

void mlink::printHeartbeatStats(){
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
    // If the system ID is new, add it to the map. Return the position of the new or existing element
    ret = sysID_stats.insert(std::pair<uint8_t,heartbeat_stats>(sysID,heartbeat_stats()));
    iter = ret.first;

    // Record when the heartbeat was received
    boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();

    // Update the map for this system ID
    iter->second.num_heartbeats_received++;
    iter->second.last_heartbeat_time = nowTime;

    // Check whether the system ID is new and log if it is
    if (ret.second == true)
      LOG(INFO) << "Adding sysID: " << (int)sysID << " to the mapping on link: " << info.link_name;
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
