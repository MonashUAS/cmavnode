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

bool mlink::onMessageRecv(mavlink_message_t *msg)
{
    recentPacketCount++;

    if(msg->msgid == MAVLINK_MSG_ID_HEARTBEAT)
        onHeartbeatRecv(msg->sysid);
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
            // Log then erase
            LOG(INFO) << "Removing sysID: " << (int)(iter->first) << " from the mapping on link: " << info.link_name;
            sysID_stats.erase(iter);
        }
    }
}
