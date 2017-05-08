#ifndef MAVHELPER_H
#define MAVHELPER_H

void getTargets(const mavlink_message_t* msg, int16_t &sysid, int16_t &compid)
{
    /* --------METHOD TAKEN FROM ARDUPILOT ROUTING LOGIC CODE ------------*/
    const mavlink_msg_entry_t *msg_entry = mavlink_get_msg_entry(msg->msgid);
    if (msg_entry == nullptr) {
        return;
    }
    if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) {
        sysid = _MAV_RETURN_uint8_t(msg,  msg_entry->target_system_ofs);
    }
    if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT) {
        compid = _MAV_RETURN_uint8_t(msg,  msg_entry->target_component_ofs);
    }
}

#endif
