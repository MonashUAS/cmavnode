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
#include "mavhelper.h"

//Periodic function timings
#define MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS 10

// Functions in this file
boost::program_options::options_description add_program_options(std::string &filename, bool &shellen, bool &verbose);
int try_user_options(int argc, char** argv, boost::program_options::options_description desc);
void runMainLoop(std::vector<std::shared_ptr<mlink> > *links, bool &verbose);
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
        std::cout << "No Valid Links found" << std::endl;
        return 1; // Catch other errors
    }

    std::cout << "Command line arguments parsed succesfully." << std::endl;
    std::cout << "Links Initialized, routing loop starting." << std::endl;

    // Number the links
    for (uint16_t i = 0; i < links.size(); i++)
    {
        links.at(i)->link_id = i;
    }

    //assign pass through link id's so we can address them fast in main loop
    for(uint16_t i = 0; i < links.size(); i++)
    {
        if(links.at(i)->info.passthrough)
        {
            //then search for matching link
            bool found = false;
            std::string passthrough_to_ = links.at(i)->info.passthrough_to;
            for(int k = 0; k < links.size(); k++)
            {
                if(!passthrough_to_.compare(links.at(k)->info.link_name))
                    {
                        found = true;
                        // this is the link you are looking for
                        links.at(i)->info.passthrough_to_id = k;
                        std::cout << "the link " << links.at(i)->info.link_name << " is being passed directly through to " << links.at(i)->info.passthrough_to << " which has id " << links.at(i)->info.passthrough_to_id << std::endl;
                        break;
                    }
            }
            if(!found) std::cout << links.at(i)->info.link_name << " has an invalid passthrough setting" << std::endl;
        }
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
    std::cout << "Links deallocated, stack unwound, exiting." << std::endl;
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
    // If the packet came from this link, don't bother
    if (outgoing_link == incoming_link)
    {
        return false;
    }

    // Don't forward SiK radio info
    if ((*incoming_link)->info.SiK_radio && msg.sysid == 51)
    {
        return false;
    }

    // If passthrough is enabled, see if this is the right link otherwise drop
    if((*incoming_link)->info.passthrough)
    {
        if((*incoming_link)->info.passthrough_to_id == (*outgoing_link)->link_id)
        {
            //then this is the specified link to pass through to
            return true;
        }
        else
        {
            //this link has been specified for passthrough but this is the wrong link
            return false;
        }
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

    // heartbeats are always forwarded
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        return true;
    }

    int16_t sysIDmsg = -1;
    int16_t compIDmsg = -1;
    getTargets(&msg, sysIDmsg, compIDmsg);
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

    // if we get this far then the packet is routable; if we can't
    // find a route for it then we drop the message.
    if (!((*outgoing_link)->seenSysID(sysIDmsg)))
    {
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

void exitGracefully(int a)
{
    std::cout << "Exit code " << a << std::endl;
    std::cout << "SIGINT caught, deconstructing links and exiting" << std::endl;
    exitMainLoop = true;
}
