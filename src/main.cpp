/* CMAVNode
 * Monash UAS
 */

#include <iostream>
#include <boost/program_options.hpp>
#include <stdio.h>
#include <signal.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include "../include/mavlink/ardupilotmega/mavlink.h"
#include "../include/logging/src/easylogging++.h"
#include <libconfig.h++>

using namespace libconfig;

#include "mlink.h"
#include "asyncsocket.h"
#include "serial.h"
#include "exception.h"


//factory to build the links
std::vector<std::unique_ptr<mlink>> linkFactory(std::vector<std::string> socketInitList, std::vector<std::string> serialInitList);

bool exitMainLoop = false;

//timing stuff, not happy with this implementation
long now_ms = 0;
long last_update_sysid_ms = 0;
long myclock();

void runMainLoop(std::vector<std::unique_ptr<mlink>> *links);
void runPeriodicFunctions(std::vector<std::unique_ptr<mlink>> *links);

//Helper function to find targets in all the message types
void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid);

std::vector<std::string> socketInitList;
std::vector<std::string> serialInitList;

//Periodic function timings
#define UPDATE_SYSID_INTERVAL_MS 10
#define MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS 10

void exitGracefully(int a)
{
    LOG(INFO) << "SIGINT caught, deconstructing links and exiting";
    exitMainLoop = true;
}

namespace
{
const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;

} // namespace

INITIALIZE_EASYLOGGINGPP

int main(int argc, char** argv)
{
    START_EASYLOGGINGPP(argc, argv);
    el::Loggers::configureFromGlobal("../conf/log.conf");
    signal(SIGINT, exitGracefully);
    try
    {
        /** Define and parse the program options
        */
        boost::program_options::options_description desc("Options");
        desc.add_options()
        ("help", "Print help messages")
        ("file", boost::program_options::value<std::vector<std::string>>(), "configuration file, usage: --file=path/to/file.conf")
        ("socket", boost::program_options::value<std::vector<std::string>>(),"UDP Link, usage: --socket=<targetip>:<targetport>:<listeningport>")
        ("serial", boost::program_options::value<std::vector<std::string>>(),"Serial Link, usage: --serial=<port>:<baudrate");

        boost::program_options::variables_map vm;
        try
        {
            boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc),
                                          vm); // can throw

            /** --help option
            */
            if ( vm.count("help")  )
            {
                std::cout << "Basic Command Line Parameter App" << std::endl
                          << desc << std::endl;
                return SUCCESS;
            }

            boost::program_options::notify(vm); // throws on error, so do after help in case
            // there are any problems

            if( !vm.count("socket") && !vm.count("serial") && !vm.count("file") )
            {
                LOG(ERROR) << "Program cannot be run without arguments.";
                std::cerr << desc << std::endl;
                return ERROR_IN_COMMAND_LINE;
            }

            Config cfg;

            try
            {
                cfg.readFile("cmavnode.cfg");
            }
            catch(const FileIOException &fioex)
            {
                LOG(ERROR) << "Cannot open config file";
                return ERROR_IN_COMMAND_LINE;
            }
            catch(const ParseException &pex)
            {
                LOG(ERROR) << "Cannot parse config file";
                return ERROR_IN_COMMAND_LINE;
            }

            const Setting& root = cfg.getRoot();

            try
            {
                const Setting &links = root["links"];
                int numlinks = links.getLength();

                LOG(INFO) << "Config file parsed, " << numlinks << " links found.";

                for(int i = 0; i < numlinks; ++i)
                {
                    bool valid = false;
                    const Setting &link = links[i];

                    std::string link_name;
                    int receive_from, output_to, output_only_from, output_only_heartbeat_from;

                    if(!(link.lookupValue("link_name", link_name)
                                && link.lookupValue("receive_from", receive_from)
                                && link.lookupValue("output_to", output_to)
                                && link.lookupValue("output_only_from", output_only_from)
                                && link.lookupValue("output_only_heartbeat_from", output_only_heartbeat_from)))
                    {
                        LOG(ERROR) << "Invalid link, ignoring";
                        continue;
                    }

                    try
                    {
                        const Setting &socket = link["socket"];
                        std::string target_ip;
                        int target_port, receive_port;

                        if((socket.lookupValue("target_ip",target_ip)
                                    && socket.lookupValue("target_port",target_port)
                                    && socket.lookupValue("receive_port",receive_port)))
                        {
                            valid = true;
                        }
                        else
                        {
                            LOG(ERROR) << "Invalid link, ignoring";
                            continue;
                        }
                            
                    }
                    catch(const SettingNotFoundException &nfex)
                    {
                        try
                        {
                        const Setting &serial = link["serial"];
                        std::string port;
                        int baud;

                        if((serial.lookupValue("port",port)
                                    && serial.lookupValue("baud",baud)))
                            valid = true;
                        else
                        {
                            LOG(ERROR) << "Invalid link, ignoring";
                            continue;
                        }
                        }
                        catch(const SettingNotFoundException &nfex)
                        {
                            LOG(ERROR) << "Invalid link, ignoring";
                            continue;
                        }
                    }
                    if(valid) LOG(INFO) << "Valid link found";
                }

            }
            catch(const SettingNotFoundException &nfex)
            {
                LOG(ERROR) << "Cannot find links in config file";
                return ERROR_IN_COMMAND_LINE;

            }

            //store link strings
            if ( vm.count("socket"))
            {
                socketInitList = vm["socket"].as<std::vector<std::string>>();
            }

            if ( vm.count("serial"))
            {
                serialInitList = vm["serial"].as<std::vector<std::string>>();
            }

        }
        catch(boost::program_options::error& e)
        {
            LOG(ERROR) << "ERROR: " << e.what();
            std::cerr << desc << std::endl;
            return ERROR_IN_COMMAND_LINE;
        }
        /*--------------END COMMAND LINE PARSING------------------*/

        LOG(INFO) << "Command line arguments parsed succesfully";

        //local object holds pointers to the links
        std::vector<std::unique_ptr<mlink>> links;

        //Set up the links
        links = linkFactory(socketInitList, serialInitList);

        LOG(INFO) << "Links Initialized";

        while(!exitMainLoop)
        {
            runMainLoop(&links);
        }

        /*----------------END MAIN CODE------------------*/
    }
    catch(std::exception& e)
    {
        LOG(FATAL) << "Unhandled Exception reached the top of main: "
                   << e.what() << ", application will now exit";
        return ERROR_UNHANDLED_EXCEPTION;

    }
    LOG(INFO) << "Links deallocated, stack unwound, exiting";
    return SUCCESS;
} //main

std::vector<std::unique_ptr<mlink>> linkFactory(std::vector<std::string> socketInitList, std::vector<std::string> serialInitList)
{
    // this function reads the strings passed in by the user, and creates the mlink objects
    // returns vector of all comms links

    std::vector<std::unique_ptr<mlink>> links;

    for(int i = 0; i < socketInitList.size(); i++)
    {
        std::vector<std::string> thisLinkArgs;
        boost::split(thisLinkArgs, socketInitList.at(i), boost::is_any_of(":"));

        if(thisLinkArgs.size() != 3)
        {
            throw Exception("Socket connection string not valid");
        }
        //create on the heap and add a pointer
        links.push_back(std::unique_ptr<mlink>(new asyncsocket(thisLinkArgs.at(0),thisLinkArgs.at(1),thisLinkArgs.at(2), i, socketInitList.at(i))));
    }

    for(int i = 0; i < serialInitList.size(); i++)
    {
        std::vector<std::string> thisLinkArgs;
        boost::split(thisLinkArgs, serialInitList.at(i), boost::is_any_of(":"));

        if(thisLinkArgs.size() != 2)
        {
            throw Exception("Serial connection string not valid");
        }
        //create on the heap and add a pointer
        links.push_back(std::unique_ptr<mlink>(new serial(thisLinkArgs.at(0),thisLinkArgs.at(1),socketInitList.size() + i ,serialInitList.at(i))));
    }

    return links;
}

void runMainLoop(std::vector<std::unique_ptr<mlink>> *links)
{
//Gets run in a while loop once links are setup

    runPeriodicFunctions(links);

    for(int i = 0; i < links->size(); i++)
    {
        mavlink_message_t msg;
        //while reading off buffer for link i
        while(links->at(i)->qReadIncoming(&msg))
        {
            int16_t sysIDmsg = 0;
            int16_t compIDmsg = 0;
            getTargets(&msg, sysIDmsg, compIDmsg);

            //we have got a message, work out where to send it
            LOG(DEBUG) << "Message received from sysID: " << (int)msg.sysid << " msgID: " << (int)msg.msgid << " target system: " << (int)sysIDmsg;

            bool wasForwarded = false;
            if(sysIDmsg == 0 || sysIDmsg == -1)
            {
                //Then message is broadcast, iterate through links
                for(int n = 0; n < links->size(); n++)
                {

                    bool sysOnThisLink = false;
                    //if the packet came from this link, dont bother
                    if(n == i) sysOnThisLink = true;
                    else   //check the routing table to see if the system is on this link
                    {
                        for(int k = 0; k < links->at(n)->sysIDpub.size(); k++)
                        {
                            //if the system that sent this message is on the list,
                            //dont send down this link
                            if(msg.sysid == links->at(n)->sysIDpub.at(k))
                            {
                                sysOnThisLink = true;
                            }
                        }
                    }

                    //If this link doesn't point to the system that sent the message, send here
                    if(!sysOnThisLink)
                    {
                        links->at(n)->qAddOutgoing(msg);
                        wasForwarded = true;
                    }
                }
            } //end broadcast block
            else
            {
                //msg is targeted
                for(int n = 0; n < links->size(); n++)
                {
                    //iterate routing table, if target is there, send
                    for(int k = 0; k < links->at(n)->sysIDpub.size(); k++)
                    {
                        if(sysIDmsg == links->at(n)->sysIDpub.at(k))
                        {
                            //then forward down this link
                            links->at(n)->qAddOutgoing(msg);
                            wasForwarded = true;
                        }
                    }
                }
            } //end targeted block

            if(!wasForwarded)
            {
                LOG(ERROR) << "Packet dropped from sysID: " << (int)msg.sysid << " msgID: " << (int)msg.msgid << " target system: " << (int)sysIDmsg;
            }
        }
    }


    boost::this_thread::sleep(boost::posix_time::milliseconds(MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS));
}

void runPeriodicFunctions(std::vector<std::unique_ptr<mlink>> *links)
{

    now_ms = myclock();

    if(now_ms - last_update_sysid_ms > UPDATE_SYSID_INTERVAL_MS)
    {

        last_update_sysid_ms = myclock();
        for(int i = 0; i < links->size(); i++)
        {
            links->at(i)->getSysID_thisLink();
        }
    }
}

void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid)
{
    /* --------METHOD TAKEN FROM ARDUPILOT ROUTING LOGIC CODE ------------*/
    // unfortunately the targets are not in a consistent position in
    // the packets, so we need a switch. Using the single element
    // extraction functions (which are inline) makes this a bit faster
    // than it would otherwise be.
    // This list of messages below was extracted using:
    //
    // cat ardupilotmega/*h common/*h|egrep
    // 'get_target_system|get_target_component' |grep inline | cut
    // -d'(' -f1 | cut -d' ' -f4 > /tmp/targets.txt
    //
    // TODO: we should write a python script to extract this list
    // properly

    switch (msg->msgid)
    {
        // these messages only have a target system
    case MAVLINK_MSG_ID_CAMERA_FEEDBACK:
        sysid = mavlink_msg_camera_feedback_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_CAMERA_STATUS:
        sysid = mavlink_msg_camera_status_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
        sysid = mavlink_msg_change_operator_control_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_SET_MODE:
        sysid = mavlink_msg_set_mode_get_target_system(msg);
        break;
    case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
        sysid = mavlink_msg_set_gps_global_origin_get_target_system(msg);
        break;

        // these support both target system and target component
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
        sysid  = mavlink_msg_digicam_configure_get_target_system(msg);
        compid = mavlink_msg_digicam_configure_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
        sysid  = mavlink_msg_digicam_control_get_target_system(msg);
        compid = mavlink_msg_digicam_control_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
        sysid  = mavlink_msg_fence_fetch_point_get_target_system(msg);
        compid = mavlink_msg_fence_fetch_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_FENCE_POINT:
        sysid  = mavlink_msg_fence_point_get_target_system(msg);
        compid = mavlink_msg_fence_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
        sysid  = mavlink_msg_mount_configure_get_target_system(msg);
        compid = mavlink_msg_mount_configure_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        sysid  = mavlink_msg_mount_control_get_target_system(msg);
        compid = mavlink_msg_mount_control_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_STATUS:
        sysid  = mavlink_msg_mount_status_get_target_system(msg);
        compid = mavlink_msg_mount_status_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT:
        sysid  = mavlink_msg_rally_fetch_point_get_target_system(msg);
        compid = mavlink_msg_rally_fetch_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_RALLY_POINT:
        sysid  = mavlink_msg_rally_point_get_target_system(msg);
        compid = mavlink_msg_rally_point_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
        sysid  = mavlink_msg_set_mag_offsets_get_target_system(msg);
        compid = mavlink_msg_set_mag_offsets_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_COMMAND_INT:
        sysid  = mavlink_msg_command_int_get_target_system(msg);
        compid = mavlink_msg_command_int_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        sysid  = mavlink_msg_command_long_get_target_system(msg);
        compid = mavlink_msg_command_long_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        sysid  = mavlink_msg_file_transfer_protocol_get_target_system(msg);
        compid = mavlink_msg_file_transfer_protocol_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        sysid  = mavlink_msg_gps_inject_data_get_target_system(msg);
        compid = mavlink_msg_gps_inject_data_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_ERASE:
        sysid  = mavlink_msg_log_erase_get_target_system(msg);
        compid = mavlink_msg_log_erase_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        sysid  = mavlink_msg_log_request_data_get_target_system(msg);
        compid = mavlink_msg_log_request_data_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        sysid  = mavlink_msg_log_request_end_get_target_system(msg);
        compid = mavlink_msg_log_request_end_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        sysid  = mavlink_msg_log_request_list_get_target_system(msg);
        compid = mavlink_msg_log_request_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ACK:
        sysid  = mavlink_msg_mission_ack_get_target_system(msg);
        compid = mavlink_msg_mission_ack_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        sysid  = mavlink_msg_mission_clear_all_get_target_system(msg);
        compid = mavlink_msg_mission_clear_all_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_COUNT:
        sysid  = mavlink_msg_mission_count_get_target_system(msg);
        compid = mavlink_msg_mission_count_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ITEM:
        sysid  = mavlink_msg_mission_item_get_target_system(msg);
        compid = mavlink_msg_mission_item_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        sysid  = mavlink_msg_mission_item_int_get_target_system(msg);
        compid = mavlink_msg_mission_item_int_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST:
        sysid  = mavlink_msg_mission_request_get_target_system(msg);
        compid = mavlink_msg_mission_request_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        sysid  = mavlink_msg_mission_request_list_get_target_system(msg);
        compid = mavlink_msg_mission_request_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
        sysid  = mavlink_msg_mission_request_partial_list_get_target_system(msg);
        compid = mavlink_msg_mission_request_partial_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        sysid  = mavlink_msg_mission_set_current_get_target_system(msg);
        compid = mavlink_msg_mission_set_current_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
        sysid  = mavlink_msg_mission_write_partial_list_get_target_system(msg);
        compid = mavlink_msg_mission_write_partial_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        sysid  = mavlink_msg_param_request_list_get_target_system(msg);
        compid = mavlink_msg_param_request_list_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        sysid  = mavlink_msg_param_request_read_get_target_system(msg);
        compid = mavlink_msg_param_request_read_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_SET:
        sysid  = mavlink_msg_param_set_get_target_system(msg);
        compid = mavlink_msg_param_set_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_PING:
        sysid  = mavlink_msg_ping_get_target_system(msg);
        compid = mavlink_msg_ping_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        sysid  = mavlink_msg_rc_channels_override_get_target_system(msg);
        compid = mavlink_msg_rc_channels_override_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        sysid  = mavlink_msg_request_data_stream_get_target_system(msg);
        compid = mavlink_msg_request_data_stream_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA:
        sysid  = mavlink_msg_safety_set_allowed_area_get_target_system(msg);
        compid = mavlink_msg_safety_set_allowed_area_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        sysid  = mavlink_msg_set_attitude_target_get_target_system(msg);
        compid = mavlink_msg_set_attitude_target_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        sysid  = mavlink_msg_set_position_target_global_int_get_target_system(msg);
        compid = mavlink_msg_set_position_target_global_int_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        sysid  = mavlink_msg_set_position_target_local_ned_get_target_system(msg);
        compid = mavlink_msg_set_position_target_local_ned_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_V2_EXTENSION:
        sysid  = mavlink_msg_v2_extension_get_target_system(msg);
        compid = mavlink_msg_v2_extension_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_REPORT:
        sysid  = mavlink_msg_gimbal_report_get_target_system(msg);
        compid = mavlink_msg_gimbal_report_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_CONTROL:
        sysid  = mavlink_msg_gimbal_control_get_target_system(msg);
        compid = mavlink_msg_gimbal_control_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT:
        sysid  = mavlink_msg_gimbal_torque_cmd_report_get_target_system(msg);
        compid = mavlink_msg_gimbal_torque_cmd_report_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK:
        sysid  = mavlink_msg_remote_log_data_block_get_target_system(msg);
        compid = mavlink_msg_remote_log_data_block_get_target_component(msg);
        break;
    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        sysid  = mavlink_msg_remote_log_block_status_get_target_system(msg);
        compid = mavlink_msg_remote_log_block_status_get_target_component(msg);
        break;
    }
}

long myclock()
{
    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::duration<float, std::milli> duration;

    static clock::time_point start = clock::now();
    duration elapsed = clock::now() - start;
    return elapsed.count();
}
