#ifndef MAVHELPER_H
#define MAVHELPER_H

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

#endif
