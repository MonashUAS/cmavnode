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

//Periodic function timings
#define MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS 10

// Functions in this file
boost::program_options::options_description add_program_options(std::string &filename, bool &shellen, bool &verbose, int &headlessport);
int try_user_options(int argc, char** argv, boost::program_options::options_description desc);
void runMainLoop(std::vector<std::shared_ptr<mlink> > *links, bool &verbose);
void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid);
void exitGracefully(int a);

bool exitMainLoop = false;

int main(int argc, char** argv)
{
    signal(SIGINT, exitGracefully);
    // Keep track of all known links
    std::vector<std::shared_ptr<mlink> > links;
    // Default mode selections
    bool shellen = true;
    bool verbose = false;
    int headlessport = -1;

    std::string filename;
    boost::program_options::options_description desc = add_program_options(filename, shellen, verbose, headlessport);

    int ret = try_user_options(argc, argv, desc);
    if (ret == 1)
        return 1; // Error
    else if (ret == -1)
        return 0; // Help option

    CmavServer headlessServer(8000, &links);
    ret = readConfigFile(filename, links);
    if (links.size() == 0)
    {
        std::cout << "No Valid Links found" << std::endl;
        return 1; // Catch other errors
    }

    std::cout << "Command line arguments parsed succesfully." << std::endl;
    std::cout << "Links Initialized, routing loop starting." << std::endl;

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
    if( !vm.count("socket") && !vm.count("serial") && !vm.count("file") )
    {
        std::cerr << "Program cannot be run without arguments." << std::endl;
        std::cerr << desc << std::endl;
        return 1; // Error in command line
    }
    return 0; // No errors or help option detected

}

bool should_forward_message(mavlink_message_t &msg, std::shared_ptr<mlink> *incoming_link, std::shared_ptr<mlink> *outgoing_link)
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
    int16_t sysIDmsg = -1;
    int16_t compIDmsg = -1;
    getTargets(&msg, sysIDmsg, compIDmsg);

    //If it is broadcast, forward
    if (sysIDmsg == -1)
    {
        return true;
    }
    if (compIDmsg == -1)
    {
        return true;
    }
    if (sysIDmsg == 0)
    {
        return true;
    }

    // packet is targeted, if the target is on this link send it
    if ((*outgoing_link)->seenSysID(sysIDmsg))
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
            int16_t sysIDmsg = -1;
            int16_t compIDmsg = -1;
            getTargets(&msg, sysIDmsg, compIDmsg);


            // Iterate through each link to send to the correct target
            for (auto outgoing_link = links->begin(); outgoing_link != links->end(); ++outgoing_link)
            {
                // mavlink routing.  See comment in MAVLink_routing.cpp
                // for logic
                if (!should_forward_message(msg, &(*incoming_link), &(*outgoing_link)))
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
                              << " target system: " << (int)sysIDmsg
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
    exitMainLoop = true;
}
