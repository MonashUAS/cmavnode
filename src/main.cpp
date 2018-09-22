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
#include "cmavserver.h"
#include "blockxmit.h"
#include "linkmanager.h"
#include "routing.h"

//Periodic function timings
#define MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS 5
#define CACHE_UPDATE_EVERY_MS 100

// Functions in this file
boost::program_options::options_description add_program_options(bool &verbose, int &headlessport, std::string &rx_dir, std::string &startup_config);
int tryUserOptions(int argc, char** argv, boost::program_options::options_description desc);
bool runMainLoop(links_t &links,source_map_t source_map_,routing_table_t routing_table_,std::shared_ptr<blockXmit> block_xmit_, bool &verbose);
void exitGracefully(int a);

bool exit_main_loop = false;

int main(int argc, char** argv)
{
    signal(SIGINT, exitGracefully);

    // variables to populate from arguments
    bool verbose = false;
    int server_port = -1;
    std::string rx_dir;
    std::string startup_config;

    // lock to protect the links vector
    std::mutex links_access_lock;

    boost::program_options::options_description desc = add_program_options(verbose, server_port, rx_dir, startup_config);

    if(tryUserOptions(argc, argv, desc) != 0)
        return 0;

    source_map_t source_map = buildSourceMap();
    routing_table_t routing_table = buildRoutingTable();

    // Allocate key s
    links_t links;
    auto block_xmit = std::make_shared<blockXmit>(rx_dir);
    auto link_manager = std::make_shared<LinkManager>(&links,std::ref(links_access_lock));
    auto json_api = std::make_shared<JsonApi>(link_manager,block_xmit,source_map,routing_table,std::ref(links_access_lock));

    if(startup_config.size() > 0)
        json_api->parseFile(startup_config);

    std::shared_ptr<CmavServer> cmav_server;
    if(server_port != -1)
        cmav_server = std::make_shared<CmavServer>( server_port, json_api );

    if (links.size() == 0)
        std::cout << "Warning: cmavnode started with no links" << std::endl;

    std::cout << "Command line arguments parsed succesfully." << std::endl;
    std::cout << "Links Initialized, routing loop starting." << std::endl;

    // Start the main loop
    while (!exit_main_loop)
    {
        static boost::posix_time::ptime last_cache_update;
        boost::posix_time::ptime now_cache = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration diff_cache = now_cache - last_cache_update;
        if(diff_cache.total_milliseconds() > CACHE_UPDATE_EVERY_MS)
            link_manager->updateLinksCache();

        bool should_sleep = false;

        {
            // this scope is to ensure the lock is released before this thread sleeps
            // otherwise other threads will never get the lock
            std::lock_guard<std::mutex> lock(links_access_lock);


            if(runMainLoop(links,source_map,routing_table,block_xmit,verbose))
            {
                should_sleep = true;
            }

            static boost::posix_time::ptime last_block_xmit_update;
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration diff = now - last_block_xmit_update;
            if(diff.total_milliseconds() > BLOCK_XMIT_RUN_EVERY_MS)
            {
                last_block_xmit_update = boost::posix_time::microsec_clock::local_time();
                // Block xmit transmit
                for (auto it : links)
                {
                    auto this_link = it.second;
                    if(this_link->info.blockXmitTx)
                    {
                        mavlink_message_t msg;
                        for(int i = 0; i < BLOCK_XMIT_CHUNKS_PER_RUN; i++)
                        {
                            if(block_xmit->sendChunk(msg))
                            {
                                this_link->qAddOutgoing(msg);
                            }
                        }
                    }
                }
            }
        }

        if(should_sleep)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(MAIN_LOOP_SLEEP_QUEUE_EMPTY_MS));
        }
    }

    // Report successful exit from main()
    std::cout << "Links deallocated, stack unwound, exiting." << std::endl;
    return 0;
}

boost::program_options::options_description add_program_options(bool &verbose, int &headlessport, std::string &rx_dir, std::string &startup_config)
{
    boost::program_options::options_description desc("Options");
    desc.add_options()
    ("help", "Print help messages")
    ("headless,H", boost::program_options::value<int>(&headlessport), "run cmavnode headless with json server, usage --headless <port>")
    ("rx_dir,r", boost::program_options::value<std::string>(&rx_dir), "Specify rx directory for file transfer, usage --rx_dir <path>")
    ("file,f", boost::program_options::value<std::string>(&startup_config), "Specify a json file to read in at startup, usage --startup_config <path_to_config>")
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

    return 0; // No errors or help option detected

}

bool runMainLoop(links_t &links,source_map_t source_map_,routing_table_t routing_table_, std::shared_ptr<blockXmit> block_xmit_,bool &verbose)
{
    // Gets run in a while loop once links are setup

    // Iterate through each link
    mavlink_message_t msg;
    bool should_sleep = true;
    for (auto it : links)
    {
        auto incoming_link = it.second;
        // Clear out dead links
        incoming_link->checkForDeadSysID();

        // Try to read from the buffer for this link
        while (incoming_link->qReadIncoming(&msg))
        {
            should_sleep = false;

            //TODO: this is dirty do better
            //block xmit receive stuff
            if(incoming_link->info.blockXmitRx &&
                    msg.msgid == MAVLINK_MSG_ID_DATA64 &&
                    msg.sysid == BLOCK_XMIT_SYSID_TX)
            {
                mavlink_message_t ack;
                block_xmit_->handleChunk(msg,ack);
                incoming_link->qAddOutgoing(ack);
            }
            else if(incoming_link->info.blockXmitTx &&
                    msg.msgid == MAVLINK_MSG_ID_DATA16 &&
                    msg.sysid == BLOCK_XMIT_SYSID_RX)
            {
                block_xmit_->handleAck(msg);
            }
            else
                routePacket(links,routing_table_,source_map_,msg,it.first);

            //else if(routePacket(links,routing_table_,source_map_,msg,it.first) < 0)
            //    std::cout << "Packet from " << (int)msg.sysid << " not routed, id: " << (int)msg.msgid << std::endl;
        }
    }

    return should_sleep;
}

void exitGracefully(int a)
{
    std::cout << "Exit code " << a << std::endl;
    std::cout << "SIGINT caught, deconstructing links and exiting" << std::endl;
    exit_main_loop = true;
}
