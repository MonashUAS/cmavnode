#include "routing.h"

source_map_t buildSourceMap()
{
    source_map_t source_map_;
    source_map_.push_back(sys_pair(1,255));
    source_map_.push_back(sys_pair(255,1));
    return source_map_;
}

routing_table_t buildRoutingTable()
{
    routing_table_t routing_table_;
    routing_table_.push_back(route(1,0));
    routing_table_.push_back(route(255,1));
}

int routePacket(links_t links_, routing_table_t routing_table_, source_map_t source_map_, mavlink_message_t &msg)
{
    int16_t sys_id_msg = -1;
    int16_t comp_id_msg = -1;
    getTargets(&msg, sys_id_msg, comp_id_msg);
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
