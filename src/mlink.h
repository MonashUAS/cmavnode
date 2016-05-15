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
#include <iostream>
#include <tuple>

#include "exception.h"

#define MAV_INCOMING_LENGTH 1000 
#define MAV_OUTGOING_LENGTH 1000
#define OUT_QUEUE_EMPTY_SLEEP 50
#define MAV_INCOMING_BUFFER_LENGTH 2041
#define MAV_HEARTBEAT_TIMEOUT_MS 1000

class mlink
{
    public:
        mlink();
        ~mlink();

        //Send or read mavlink messages
        void qAddOutgoing(mavlink_message_t msg);
        bool qReadIncoming(mavlink_message_t *msg);

        //update the public mapping based on private one
        void getSysID_thisLink();

        //remove dead systems from private mapping
        void checkForDeadSysID();
        

        void onHeartbeatRecv(uint8_t sysID);


        //Public system ID mapping
        std::vector<uint8_t> sysIDpub;
       
        //Read and write thread functions. Read thread will call ioservice.run and block
        //Write thread will be in an infinate busy wait loop
        virtual void runWriteThread(){};
        virtual void runReadThread(){};

        
        //To identify links for debugging. string stores the raw string the port was opened with.
        int linkID;
        std::string rawString;

    protected:
        //Keep track of system ID's on this link and the last heartbeat time
        std::vector<std::tuple<uint8_t, boost::posix_time::ptime>> sysID_thisLink;

        boost::lockfree::spsc_queue<mavlink_message_t> qMavIn{MAV_INCOMING_LENGTH};
        boost::lockfree::spsc_queue<mavlink_message_t> qMavOut{MAV_OUTGOING_LENGTH};

        boost::thread read_thread;
        boost::thread write_thread;

        bool exitFlag = false;

        uint8_t data_in_[MAV_INCOMING_BUFFER_LENGTH];
        uint8_t data_out_[MAV_INCOMING_BUFFER_LENGTH];
};

#endif
