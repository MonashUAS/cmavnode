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
    std::vector<uint8_t> mapping;
    for(int i = 0; i < sysID_thisLink.size(); i++)
    {
        mapping.push_back(std::get<0>(sysID_thisLink.at(i)));
        //LOG(INFO) << "Adding sysid " << std::get<0>(sysID_thisLink.at(i)) << "to public mapping";
    }

    sysIDpub = mapping;
}

void mlink::onMessageRecv(mavlink_message_t *msg)
{
    //Check if this message needs special handling based on content
    
    recentPacketCount++;

    if(msg->msgid == MAVLINK_MSG_ID_HEARTBEAT)
        onHeartbeatRecv(msg->sysid);
}

void mlink::printHeartbeatStats(){
    std::cout << "HEARTBEAT STATS FOR LINK: " << info.link_name << std::endl;

    for(int i = 0; i < heartbeattracker.size();i++)
    {
        std::cout << "sysID: " << (int)heartbeattracker.at(i).first 
            << " # heartbeats: " << heartbeattracker.at(i).second << std::endl;
    }
}

void mlink::onHeartbeatRecv(uint8_t sysID)
{
    //heartbeat tracker
    
    bool heartbeatexists = false;
    for(int i = 0; i < heartbeattracker.size();i++)
    {
        if(heartbeattracker.at(i).first == sysID){
            heartbeatexists = true;
            heartbeattracker.at(i).second++;
        }
    }
    if(!heartbeatexists){
        std::pair<uint8_t, int> tmp(sysID,1);
        heartbeattracker.push_back(tmp);
    }


    bool exists = false;
    uint8_t indexIfExists;

    for(int i = 0; i < sysID_thisLink.size(); i++)
    {
        if(std::get<0>(sysID_thisLink.at(i)) == sysID)
        {
            exists = true;
            indexIfExists = i;
        }
    }

    if(!exists)
    {
        //add it to the mapping
        boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();
        sysID_thisLink.push_back(std::make_tuple(sysID, nowTime));
        LOG(INFO) << "Adding sysID: " << (int)sysID << " to the mapping on link: " << info.link_name;
    }
    else
    {
        //just update the last heartbeat time
        boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();
        std::get<1>(sysID_thisLink.at(indexIfExists)) = nowTime;
    }

}


void mlink::checkForDeadSysID()
{
    //Check that no links have timed out
    //if they have, remove from mapping

    //get the time now
    boost::posix_time::ptime nowTime = boost::posix_time::microsec_clock::local_time();

    for(int i = 0; i < sysID_thisLink.size(); i++)
    {

        //tuple syntax is gross
        boost::posix_time::time_duration dur = nowTime - std::get<1>(sysID_thisLink.at(i));
        long milliseconds = dur.total_milliseconds();

        if(milliseconds > MAV_HEARTBEAT_TIMEOUT_MS)
        {
            LOG(INFO) << "Removing sysID: " << (int)std::get<0>(sysID_thisLink.at(i)) << " from the mapping on link: " << info.link_name;
            sysID_thisLink.erase(sysID_thisLink.begin() + i);

            //decrement i so we dont miss a sysID
            i--;
        }
    }

}
