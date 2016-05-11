/* CMAVNode
 * Monash UAS
 */

#include <iostream>
#include <boost/program_options.hpp>
#include <string>
#include <vector>
#include <memory>
#include <ardupilotmega/mavlink.h>

#include "mlink.h"
#include "asyncsocket.h"
#include "serial.h"
#include "exception.h"


std::vector<std::unique_ptr<mlink>> links;

std::vector<uint8_t> sysIDgMap;

std::vector<std::unique_ptr<mlink>> setupLinks(std::vector<std::string> socketInitList, std::vector<std::string> serialInitList);

std::string socketGetHost(std::string s);
std::string socketGetHostPort(std::string s);
std::string socketGetListenPort(std::string s);

void runMainLoop();
void runPeriodicFunctions();

void get_targets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid);

std::vector<std::string> socketInitList;
std::vector<std::string> serialInitList;

namespace 
{ 
  const size_t ERROR_IN_COMMAND_LINE = 1; 
  const size_t SUCCESS = 0; 
  const size_t ERROR_UNHANDLED_EXCEPTION = 2; 
 
} // namespace 

int main(int argc, char** argv)
{
try 
{ 
    /** Define and parse the program options 
    */ 
    boost::program_options::options_description desc("Options"); 
    desc.add_options() 
    ("help", "Print help messages") 
    ("socket", boost::program_options::value<std::vector<std::string>>(),"UDP Link")
    ("serial", boost::program_options::value<std::vector<std::string>>(),"Serial Link");

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

        //store link strings
        if ( vm.count("socket"))
        {
                socketInitList = vm["socket"].as<std::vector<std::string>>();
        }

        if ( vm.count("serial"))
        {
                serialInitList = vm["serial"].as<std::vector<std::string>>();
        }

        boost::program_options::notify(vm); // throws on error, so do after help in case 
        // there are any problems 

    } 
    catch(boost::program_options::error& e) 
    { 
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
        std::cerr << desc << std::endl; 
        return ERROR_IN_COMMAND_LINE; 
    } 
    /*--------------END COMMAND LINE PARSING------------------*/

    //Set up the links
    links = setupLinks(socketInitList, serialInitList); 


    while(1)
    {
        runMainLoop();
    }

    /*----------------END MAIN CODE------------------*/
} 
catch(std::exception& e) 
{ 
    std::cerr << "Unhandled Exception reached the top of main: " 
    << e.what() << ", application will now exit" << std::endl; 
    return ERROR_UNHANDLED_EXCEPTION; 

} 
} //main

std::vector<std::unique_ptr<mlink>> setupLinks(std::vector<std::string> socketInitList, std::vector<std::string> serialInitList)
{
    // this function reads the strings passed in by the user, and creates the mlink objects
    // returns vector of all comms links
    
    std::vector<std::unique_ptr<mlink>> links;

    for(int i = 0; i < socketInitList.size(); i++){
        //parse the raw connection string
        std::string hostip = socketGetHost(socketInitList.at(i));
        std::string hostport = socketGetHostPort(socketInitList.at(i));
        std::string listenport = socketGetListenPort(socketInitList.at(i));

        //create on the heap and add a pointer
        links.push_back(std::unique_ptr<mlink>(new asyncsocket(hostip,hostport,listenport)));
    }

    return links;
}

std::string socketGetHost(std::string s){
    std::string::size_type pos = s.find(':');
    if (pos != std::string::npos)
    {
        return s.substr(0, pos);
    } else {
        //error handling
    }
}

std::string socketGetHostPort(std::string s){
    std::string::size_type pos = s.find(':');
    std::string::size_type pos2 = s.find(':', pos + 1);
    if (pos2!= std::string::npos)
    {
        return s.substr(pos + 1, pos2 - (pos+1));
    } else {
        //error handling
    }
}

std::string socketGetListenPort(std::string s){
    std::string::size_type pos = s.find(':');
    std::string::size_type pos2 = s.find(':', pos + 1);
    if (pos2!= std::string::npos)
    {
        return s.substr(pos2 + 1);
    } else {
        //error handling
    }
}


void runMainLoop(){
//Gets run in a while loop once links are setup

    runPeriodicFunctions();

    for(int i = 0; i < links.size(); i++){
        mavlink_message_t msg;
        //while reading off buffer for link i
        while(links.at(i)->qReadIncoming(&msg)){
            int16_t sysIDmsg = 0;
            int16_t compIDmsg = 0;
            get_targets(&msg, sysIDmsg, compIDmsg);
            //we have got a message, work out where to send it
            std::cout << "Message received from sysID: " << (int)msg.sysid << std::endl;


            if(sysIDmsg == 0 || sysIDmsg == -1){
            //Then message is broadcast, iterate through links
                for(int n = 0; n < links.size(); n++){

                    bool sysOnThisLink = false;
                    //For each link, iterate through its routing table
                    for(int k = 0; k < links.at(n)->sysIDpub.size(); k++){
                        //if the system that sent this message is on the list,
                        //dont send down this link
                        if(msg.sysid == links.at(n)->sysIDpub.at(k)){
                            sysOnThisLink = true;
                        }
                    }

                    //If this link doesn't point to the system that sent the message, send here
                    if(!sysOnThisLink){
                        links.at(n)->qAddOutgoing(msg);
                        std::cout << "Broadcast message sent from: "<< (int)msg.sysid << std::endl;
                    }
                }
            } else {
                //msg is targeted
                for(int n = 0; n < links.size(); n++){
                    //iterate routing table, if target is there, send
                    for(int k = 0; k < links.at(n)->sysIDpub.size(); k++){
                        if(sysIDmsg == links.at(n)->sysIDpub.at(k)){
                        //then forward down this link
                        links.at(n)->qAddOutgoing(msg);
                        std::cout << "Targeted message forwarded from "<< (int)msg.sysid << " to " << (int)links.at(n)->sysIDpub.at(k) << std::endl;
                        }
                    }
                }
            }
                        

            //for(int n = 0; n < links.size(); n++){
            //    if(sysIDmsg == 0 || sysIDmsg == -1){
            //        //broadcast
            //        //send down any link that doesnt contain this sysid

            //        for(int k = 0; k < links.at(n)->sysIDpub.size(); k++){
            //            //if the messsage sysID is on this link, add to this links queue 
            //            if(n != i){
            //                if(msg.sysid != links.at(n)->sysIDpub.at(k)){
            //                //then forward down this link
            //                links.at(n)->qAddOutgoing(msg);
            //                std::cout << "targeted message forwarded from "<< (int)msg.sysid << " to " << (int)links.at(n)->sysIDpub.at(k) << std::endl;
            //                }
            //            }
            //        }

            //       // if(n != i){
            //       //     links.at(n)->qAddOutgoing(msg);
            //       //     std::cout << "broadcast message forwarded" << std::endl;
            //       // }
            //    } else{
            //        //we need to find target
            //        for(int k = 0; k < links.at(n)->sysIDpub.size(); k++){
            //            //if the messsage sysID is on this link, add to this links queue 
            //            if(n != i){
            //                if(sysIDmsg == links.at(n)->sysIDpub.at(k)){
            //                //then forward down this link
            //                links.at(n)->qAddOutgoing(msg);
            //                std::cout << "targeted message forwarded from "<< (int)msg.sysid << " to " << (int)links.at(n)->sysIDpub.at(k) << std::endl;
            //                }
            //            }
            //        }
            //    }
            //}
        }
    }
}

void runPeriodicFunctions(){

    for(int i = 0; i < links.size(); i++)
    {
        links.at(i)->getSysID_thisLink();
    }
}

void get_targets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid)
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
    
    switch (msg->msgid) {
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
