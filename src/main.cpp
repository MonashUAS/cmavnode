/* CMAVNode
 * Monash UAS
 */

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
#include "cmavserver.h"
#include "linkmanager.h"

//Periodic function timings
#define MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS 10

// Functions in this file
boost::program_options::options_description add_program_options(std::string &filename, bool &shellen, bool &verbose, int &headlessport);
int tryUserOptions(int argc, char** argv, boost::program_options::options_description desc);
void runMainLoop(std::vector<std::shared_ptr<mlink> > *links, bool &verbose);
void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid);
void exitGracefully(int a);

bool exit_main_loop = false;

bool start_with_configfile = false;

int main(int argc, char** argv)
{
    signal(SIGINT, exitGracefully);

    // variables to populate from arguments
    bool shell_enable = true;
    bool verbose = false;
    int server_port = -1;
    std::string filename;

    boost::program_options::options_description desc = add_program_options(filename, shell_enable, verbose, server_port);

    if(tryUserOptions(argc, argv, desc) != 0)
        return 0;

    // Allocate key structures
    std::vector<std::shared_ptr<mlink> > links;
    auto link_manager = std::make_shared<LinkManager>(&links);
    auto json_api = std::make_shared<JsonApi>(link_manager);

    std::shared_ptr<CmavServer> cmav_server;
    if(server_port != -1)
        cmav_server = std::make_shared<CmavServer>( server_port, json_api );

    if(start_with_configfile)
    {
        if(readConfigFile(filename, link_manager))
            return 1;
    }

    // Create links config file has queued for creation
    if(link_manager->hasPending())
        link_manager->operate();

    if (links.size() == 0)
        std::cout << "Warning: cmavnode started with no links" << std::endl;

    std::cout << "Command line arguments parsed succesfully." << std::endl;
    std::cout << "Links Initialized, routing loop starting." << std::endl;

    // Run the shell thread
    boost::thread shell_thread;
    if (shell_enable)
    {
        shell_thread = boost::thread(runShell, boost::ref(exit_main_loop), boost::ref(links));
        // The boost::thread constructor implicitly binds runShell to &exitMainLoop and &links
    }

    // Start the main loop
    while (!exit_main_loop)
    {
        if(link_manager->hasPending())
        {
            link_manager->operate();
        }
        runMainLoop(&links, verbose);
    }

    // Once the main loop is done, rejoin the shell thread
    if (shell_enable)
        shell_thread.join();

    // Report successful exit from main()
    std::cout << "Links deallocated, stack unwound, exiting." << std::endl;
    return 0;
}

boost::program_options::options_description add_program_options(std::string &filename, bool &shellen, bool &verbose, int &headlessport)
{
    boost::program_options::options_description desc("Options");
    desc.add_options()
    ("help", "Print help messages")
    ("file,f", boost::program_options::value<std::string>(&filename), "configuration file, usage: --file=path/to/file.conf")
    ("headless,H", boost::program_options::value<int>(&headlessport), "run cmavnode headless with json server, usage --headless <port>")
    ("interface,i", boost::program_options::bool_switch(&shellen), "start in interactive mode with cmav shell")
    ("verbose,v", boost::program_options::bool_switch(&verbose), "verbose output including dropped packets");
    return desc;
}

int tryUserOptions(int argc, char** argv, boost::program_options::options_description desc)
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
        std::cout << "ERROR: " << e.what() << std::endl;
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
        std::cerr << "ERROR: " << e.what() << std::endl;
        std::cerr << desc << std::endl;
        return 1; // Error in command line
    }

    // If no known option were given, return an error
    if( vm.count("file") )
    {
        start_with_configfile = true;
    }
    return 0; // No errors or help option detected

}

bool shouldForwardMessage(mavlink_message_t &msg, std::shared_ptr<mlink> *incoming_link, std::shared_ptr<mlink> *outgoing_link)
{

    // First check for reasons to drop the packet

    // If the packet came from this link drop
    if (outgoing_link == incoming_link)
    {
        return false;
    }

    // If the packet is from a SiK Radio, drop
    if ((*incoming_link)->info.SiK_radio && msg.sysid == 51)
    {
        return false;
    }

    // If the outgoing link has defined routing rules,
    // and this packet does not satisfy them, drop
    if ((*outgoing_link)->info.output_only_from[0] != 0 &&
            std::find((*outgoing_link)->info.output_only_from.begin(),
                      (*outgoing_link)->info.output_only_from.end(),
                      msg.sysid) == (*outgoing_link)->info.output_only_from.end())
    {
        return false;
    }


    // No reason to drop, now check for reasons the packet should be forwarded
    // If Heartbeat, forward
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        return true;
    }

    //Find out if message is targeted
    int16_t sys_id_msg = -1;
    int16_t comp_id_msg = -1;
    getTargets(&msg, sys_id_msg, comp_id_msg);

    //If it is broadcast, forward
    if (sys_id_msg == -1)
    {
        return true;
    }
    if (comp_id_msg == -1)
    {
        return true;
    }
    if (sys_id_msg == 0)
    {
        return true;
    }

    // packet is targeted, if the target is on this link send it
    if ((*outgoing_link)->seenSysID(sys_id_msg))
    {
        return true;
    }
    // TODO: should check sysid/compid combination has been seen, not
    // just sysid

    // packet is targeted, but the target has not been seen on this link, so drop
    return false;
}

void runMainLoop(std::vector<std::shared_ptr<mlink> > *links, bool &verbose)
{
    // Gets run in a while loop once links are setup

    // Iterate through each link
    mavlink_message_t msg;
    bool should_sleep = true;
    for (auto incoming_link = links->begin(); incoming_link != links->end(); ++incoming_link)
    {
        // Clear out dead links
        (*incoming_link)->checkForDeadSysID();

        // Try to read from the buffer for this link
        while ((*incoming_link)->qReadIncoming(&msg))
        {
            should_sleep = false;
            // Determine the correct target system ID for this message
            int16_t sys_id_msg = -1;
            int16_t comp_id_msg = -1;
            getTargets(&msg, sys_id_msg, comp_id_msg);


            // Iterate through each link to send to the correct target
            for (auto outgoing_link = links->begin(); outgoing_link != links->end(); ++outgoing_link)
            {
                // mavlink routing.  See comment in MAVLink_routing.cpp
                // for logic
                if (!shouldForwardMessage(msg, &(*incoming_link), &(*outgoing_link)))
                {
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
                    std::cout << "Packet dropped from sysID: " << (int)msg.sysid
                              << " msgID: " << (int)msg.msgid
                              << " target system: " << (int)sys_id_msg
                              << " link name: " << (*incoming_link)->info.link_name << std::endl;
                }
            }
        }
    }
    if (should_sleep)
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS));
    }
}

void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid)
{
    /* --------METHOD TAKEN FROM ARDUPILOT ROUTING LOGIC CODE ------------*/
    const mavlink_msg_entry_t *msg_entry = mavlink_get_msg_entry(msg->msgid);
    if (msg_entry == nullptr)
    {
        return;
    }
    if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM)
    {
        sysid = _MAV_RETURN_uint8_t(msg,  msg_entry->target_system_ofs);
    }
    if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT)
    {
        compid = _MAV_RETURN_uint8_t(msg,  msg_entry->target_component_ofs);
    }
}

void exitGracefully(int a)
{
    std::cout << "Exit code " << a << std::endl;
    std::cout << "SIGINT caught, deconstructing links and exiting" << std::endl;
    exit_main_loop = true;
}
