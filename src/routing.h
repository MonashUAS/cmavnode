#ifndef ROUTING_H
#define ROUTING_H

#include <vector>
#include <cstdint>

#include "mlink.h"
#include "linkmanager.h"

struct route
{
    uint8_t dest;
    int next_hop;
    route() {};
    route(uint8_t dest_,int next_hop_) : dest(dest_),next_hop(next_hop_)
    {
    };
};

struct sys_pair
{
    uint8_t src;
    uint8_t dest;
    bool bidir;
    sys_pair(uint8_t src_,uint8_t dest_,bool bidir_) : dest(dest_),src(src_),bidir(bidir_)
    {
    };
};

typedef std::shared_ptr<std::vector<sys_pair>> source_map_t;
typedef std::shared_ptr<std::vector<route>> routing_table_t;

routing_table_t buildRoutingTable();
source_map_t buildSourceMap();

int routePacket(links_t &links_, routing_table_t routing_table_, source_map_t source_map_, mavlink_message_t &msg, int incoming_link);

// Get a ist of sysids that should receive broadcast messages from this sys
std::vector<uint8_t> getSourceMapTargets(source_map_t map, uint8_t source_sysid);

// Get a list of links that this packet should be sent on
std::vector<int> getNextHop(routing_table_t &table, std::vector<uint8_t> target_systems);

void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid);

#endif
