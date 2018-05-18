#include "routing.h"

source_map_t buildSourceMap()
{
    source_map_t source_map_ = std::make_shared<std::vector<sys_pair>>();
    source_map_->push_back(sys_pair(1,255,true));
    source_map_->push_back(sys_pair(2,254,true));
    return source_map_;
}

routing_table_t buildRoutingTable()
{
    routing_table_t routing_table_ = std::make_shared<std::vector<route>>();
    routing_table_->push_back(route(1,0));
    routing_table_->push_back(route(255,1));
    return routing_table_;
}

int routePacket(links_t &links_, routing_table_t routing_table_, source_map_t source_map_, mavlink_message_t &msg, int incoming_link)
{
    // Drop SiK Radio Packets
    if (msg.sysid == 51)
    {
        return -1;
    }

    // Find targets if any
    int16_t sys_id_msg = -1;
    int16_t comp_id_msg = -1;
    getTargets(&msg, sys_id_msg, comp_id_msg);

    std::vector<uint8_t> target_systems;
    if(sys_id_msg <= 0) // route broadcast packets
    {
        target_systems = getSourceMapTargets(source_map_,msg.sysid);
    }
    else // route targeted packets
    {
        target_systems.push_back(sys_id_msg);
    }

    // If we cant find a dest sysid abort
    if(target_systems.size() == 0)
        return -1;

    std::vector<int> target_links = getNextHop(routing_table_, target_systems);

    // If we cant find a next hop abort
    if(target_links.size() == 0)
        return -1;

    // We got this far so add to queues on chosen next hops
    int wasrouted = -1;
    for(auto it : target_links)
    {
        if(links_.count(it) != 0)
        {
            links_[it]->qAddOutgoing(msg);
            wasrouted = 0;
        }
    }
    return wasrouted;
}

std::vector<uint8_t> getSourceMapTargets(source_map_t map, uint8_t source_sysid)
{
    std::vector<uint8_t> map_targets;
    for(auto it : *map)
    {
        if(it.src == source_sysid)
            map_targets.push_back(it.dest);
    }
    return map_targets;
}

std::vector<int> getNextHop(routing_table_t &table, std::vector<uint8_t> target_systems)
{
    std::vector<int> next_hop;
    for(auto it : *table)
    {
        if(std::find(target_systems.begin(), target_systems.end(), it.dest) != target_systems.end())
        {
            next_hop.push_back(it.next_hop);
        }
    }
    return next_hop;
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
