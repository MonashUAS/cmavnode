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
    sys_pair(uint8_t src_,uint8_t dest_) : dest(dest_),src(src_)
    {
    };
};

typedef std::vector<sys_pair> source_map_t;
typedef std::vector<route> routing_table_t;

routing_table_t buildRoutingTable();
source_map_t buildSourceMap();

int routePacket(links_t links_, routing_table_t routing_table_, source_map_t source_map_, mavlink_message_t &msg);
void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid);

#endif
