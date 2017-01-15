/* CMAVNode
 * Monash UAS
 */

#include "../include/logging/src/easylogging++.h"
#include <boost/program_options.hpp>
#include <string>
#include <vector>
#include <boost/thread/thread.hpp>
#include <algorithm>
#include <ostream>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <iomanip>

// CMAVNode headers
#include "mlink.h"
#include "asyncsocket.h"
#include "serial.h"
#include "exception.h"
#include "shell.h"
#include "configfile.h"

//Periodic function timings
#define MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS 10

// Functions in this file
boost::program_options::options_description add_program_options(std::string &filename, bool &shellen, bool &verbose);
int try_user_options(int argc, char** argv, boost::program_options::options_description desc);
void runMainLoop(std::vector<std::shared_ptr<mlink> > *links, bool &verbose);
void printLinkStats(std::vector<std::shared_ptr<mlink> > *links);
void printLinkQuality(std::vector<std::shared_ptr<mlink> > *links);
void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid);
void exitGracefully(int a);

bool exitMainLoop = false;

INITIALIZE_EASYLOGGINGPP

int main(int argc, char** argv)
{
    signal(SIGINT, exitGracefully);
    // Keep track of all known links
    std::vector<std::shared_ptr<mlink> > links;
    // Default mode selections
    bool shellen = true;
    bool verbose = false;

    // Begin logging
    START_EASYLOGGINGPP(argc, argv);
    el::Loggers::configureFromGlobal("log.conf");

    std::string filename;
    boost::program_options::options_description desc = add_program_options(filename, shellen, verbose);

    int ret = try_user_options(argc, argv, desc);
    if (ret == 1)
        return 1; // Error
    else if (ret == -1)
        return 0; // Help option

    ret = readConfigFile(filename, links);
    if (links.size() == 0)
    {
        LOG(ERROR) << "No Valid Links found";
        return 1; // Catch other errors
    }

    LOG(INFO) << "Command line arguments parsed succesfully.";
    LOG(INFO) << "Links Initialized, routing loop starting.";

    // Number the links
    for (uint16_t i = 0; i != links.size(); ++i)
    {
        links.at(i)->link_id = i;
    }

    // Run the shell thread
    boost::thread shell;
    if (shellen)
    {
        shell = boost::thread(runShell, boost::ref(exitMainLoop), boost::ref(links));
        // The boost::thread constructor implicitly binds runShell to &exitMainLoop and &links
    }

    // Start the main loop
    while (!exitMainLoop)
    {
        runMainLoop(&links, verbose);
    }

    // Once the main loop is done, rejoin the shell thread
    if (shellen)
        shell.join();

    // Report successful exit from main()
    LOG(INFO) << "Links deallocated, stack unwound, exiting.";
    return 0;
}

boost::program_options::options_description add_program_options(std::string &filename, bool &shellen, bool &verbose)
{
    boost::program_options::options_description desc("Options");
    desc.add_options()
    ("help", "Print help messages")
    ("file,f", boost::program_options::value<std::string>(&filename), "configuration file, usage: --file=path/to/file.conf")
    ("interface,i", boost::program_options::bool_switch(&shellen), "start in interactive mode with cmav shell")
    ("verbose,v", boost::program_options::bool_switch(&verbose), "verbose output including dropped packets");
    return desc;
}

int try_user_options(int argc, char** argv, boost::program_options::options_description desc)
{
    // Respond to the initial input (if any) provided by the user
    boost::program_options::variables_map vm;
    try
    {
        boost::program_options::store(
            boost::program_options::parse_command_line(argc, argv, desc), vm);
    }
    catch (boost::program_options::error& e)
    {
        LOG(ERROR) << "ERROR: " << e.what();
        std::cerr << desc << std::endl;
        return 1; // Error in command line
    }

    // --help option
    if (vm.count("help"))
    {
        std::cout << "CMAVNode - Mavlink router" << std::endl
                  << desc << std::endl;
        return -1; // Help option selected
    }

    // Catch potential errors again
    try
    {
        boost::program_options::notify(vm);
    }
    catch (boost::program_options::error& e)
    {
        LOG(ERROR) << "ERROR: " << e.what();
        std::cerr << desc << std::endl;
        return 1; // Error in command line
    }

    // If no known option were given, return an error
    if( !vm.count("socket") && !vm.count("serial") && !vm.count("file") )
    {
        LOG(ERROR) << "Program cannot be run without arguments.";
        std::cerr << desc << std::endl;
        return 1; // Error in command line
    }
    return 0; // No errors or help option detected

}

bool should_forward_message(mavlink_message_t &msg, std::shared_ptr<mlink> *incoming_link, std::shared_ptr<mlink> *outgoing_link)
{

    // If the packet came from this link, don't bother
    if (outgoing_link == incoming_link)
    {
        return false;
    }

    // If the current link being checked is designated to receive
    // from a non-zero system ID and that system ID isn't present on
    // this link, don't send on this link.
    if ((*outgoing_link)->info.output_only_from[0] != 0 &&
        std::find((*outgoing_link)->info.output_only_from.begin(),
                  (*outgoing_link)->info.output_only_from.end(),
                  msg.sysid) == (*outgoing_link)->info.output_only_from.end())
    {
        return false;
    }

    int16_t sysIDmsg = -1;
    int16_t compIDmsg = -1;
    getTargets(&msg, sysIDmsg, compIDmsg);
    if (sysIDmsg == -1) {
        return true;
    }
    if (compIDmsg == -1) {
        return true;
    }
    if (sysIDmsg == 0) {
        return true;
    }

    // if we get this far then the packet is routable; if we can't
    // find a route for it then we drop the message.
    if (!((*outgoing_link)->seenSysID(sysIDmsg))) {
        return false;
    }

    // TODO: should check sysid/compid combination has been seen, not
    // just sysid

    return true;
}

void runMainLoop(std::vector<std::shared_ptr<mlink> > *links, bool &verbose)
{
    // Gets run in a while loop once links are setup

    // Iterate through each link
    mavlink_message_t msg;
    for (auto incoming_link = links->begin(); incoming_link != links->end(); ++incoming_link)
    {
        // Try to read from the buffer for this link
        while ((*incoming_link)->qReadIncoming(&msg))
        {
            // Determine the correct target system ID for this message
            int16_t sysIDmsg = -1;
            int16_t compIDmsg = -1;
            getTargets(&msg, sysIDmsg, compIDmsg);

            // Use the system ID to determine where to send the message
            LOG(DEBUG) << "Message received from sysID: " << (int)msg.sysid << " msgID: " << (int)msg.msgid << " target system: " << (int)sysIDmsg;

            // Iterate through each link to send to the correct target
            for (auto outgoing_link = links->begin(); outgoing_link != links->end(); ++outgoing_link)
            {
                // mavlink routing.  See comment in MAVLink_routing.cpp
                // for logic
                if (!should_forward_message(msg, &(*incoming_link), &(*outgoing_link))) {
                    continue;
                }

                // Provided nothing else has failed and the link is up, add the
                // message to the outgoing queue.
                if ((*outgoing_link)->up)
                {
                    (*outgoing_link)->qAddOutgoing(msg);
                }
                else if (verbose)
                {
                    LOG(ERROR) << "Packet dropped from sysID: " << (int)msg.sysid
                               << " msgID: " << (int)msg.msgid
                               << " target system: " << (int)sysIDmsg
                               << " link name: " << (*incoming_link)->info.link_name;
                }
            }
        }
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS));
}

void printLinkStats(std::vector<std::shared_ptr<mlink> > *links)
{
    LOG(INFO) << "---------------------------------------------------------------------";
    // Print stats for each known link
    for (auto curr_link = links->begin(); curr_link != links->end(); ++curr_link)
    {
        std::ostringstream buffer;

        buffer << "Link: " << (*curr_link)->link_id << " "
               << (*curr_link)->info.link_name << " ";
        if ((*curr_link)->is_kill)
        {
            buffer << "DEAD ";
        }
        else if ((*curr_link)->up)
        {
            buffer << "UP ";
        }
        else
        {
            buffer << "DOWN ";
        }

        buffer << "Received: " << (*curr_link)->recentPacketCount << " "
               << "Sent: " << (*curr_link)->recentPacketSent << " "
               << "Systems on link: " << (*curr_link)->sysID_stats.size();

        // Reset the recent packet counts
        (*curr_link)->recentPacketCount = 0;
        (*curr_link)->recentPacketSent = 0;

        LOG(INFO) << buffer.str();
    }
    LOG(INFO) << "---------------------------------------------------------------------";
}

void printLinkQuality(std::vector<std::shared_ptr<mlink> > *links)
{
    // Create a line of link quality
    std::ostringstream buffer;
    for (auto curr_link = links->begin(); curr_link != links->end(); ++curr_link)
    {
        buffer << "\nLink: " << (*curr_link)->link_id
               << "   (" << (*curr_link)->info.link_name << ")\n";

        // Don't print radio stats when none have been received
        if ((*curr_link)->link_quality.link_delay +
            (*curr_link)->link_quality.local_rssi +
            (*curr_link)->link_quality.remote_rssi +
            (*curr_link)->link_quality.local_noise +
            (*curr_link)->link_quality.remote_noise +
            (*curr_link)->link_quality.rx_errors +
            (*curr_link)->link_quality.corrected_packets +
            (*curr_link)->link_quality.tx_buffer != 0)
        {
            buffer  << std::setw(17)
                    << "Link delay: "<< std::setw(5) << (*curr_link)->link_quality.link_delay << " s\n"
                    << std::setw(17)
                    << "Local RSSI: " << std::setw(5) << (*curr_link)->link_quality.local_rssi
                    << std::setw(23)
                    << "Remote RSSI: " << std::setw(5) << (*curr_link)->link_quality.remote_rssi << "\n"
                    << std::setw(17)
                    << "Local noise: " << std::setw(5) << (*curr_link)->link_quality.local_noise
                    << std::setw(23)
                    << "Remote noise: " << std::setw(5) << (*curr_link)->link_quality.remote_noise << "\n"
                    << std::setw(17)
                    << "RX errors: " << std::setw(5) << (*curr_link)->link_quality.rx_errors
                    << std::setw(23)
                    << "Corrected packets: " << std::setw(5) << (*curr_link)->link_quality.corrected_packets << "\n"
                    << std::setw(17)
                    << "TX buffer: " << std::setw(5) << (*curr_link)->link_quality.tx_buffer << "%\n\n";
        }
        if ((*curr_link)->sysID_stats.size() != 0)
            buffer << std::setw(15) <<"System ID"
                   << std::setw(19) <<"Packets Lost"
                   << std::setw(19) <<"Packets Dropped" << "\n";
        for (auto iter = (*curr_link)->sysID_stats.begin(); iter != (*curr_link)->sysID_stats.end(); ++iter)
        {
            buffer << std::setw(15) << (int)iter->first
                   << std::setw(19) << iter->second.packets_lost
                   << std::setw(19) << iter->second.packets_dropped << "\n";

            iter->second.packets_lost = 0;
            iter->second.packets_dropped = 0;
        }
    }
    LOG(INFO) << "\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
              << buffer.str()
              <<   "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";

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

void exitGracefully(int a)
{
    std::cout << "Exit code " << a << std::endl;
    LOG(INFO) << "SIGINT caught, deconstructing links and exiting";
    exitMainLoop = true;
}
