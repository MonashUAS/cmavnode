/* CMAVNode
 * Monash UAS
 *
 * JSON API layer
 */

#include "json_api.h"

namespace pt = boost::property_tree;

JsonApi::JsonApi(std::shared_ptr<LinkManager> manager)
{
    manager_ = manager;
}
JsonApi::~JsonApi()
{
    
}

std::string JsonApi::getLinks()
{
    std::vector<std::shared_ptr<MlinkCached>> cache = manager_->getLinks();
    pt::ptree jsonroot;
    pt::ptree linksroot;

    for(auto it : cache)
    {
        // TODO: factor out the conversion of properties and options into json
        int link_id_ = it->link_id_;
        LinkOptions info_ = it->link_options_;
        auto serialpointer = std::dynamic_pointer_cast<SerialCached>(it);
        auto udppointer = std::dynamic_pointer_cast<AsyncSocketCached>(it);

        pt::ptree thislinkroot;

        if(serialpointer != NULL){
            serial_properties properties_ = serialpointer->properties_;
            pt::ptree serialroot;
            serialroot.put("port", properties_.port);
            serialroot.put("baudrate", properties_.baudrate);
            serialroot.put("flowcontrol", properties_.flowcontrol);

            thislinkroot.add_child("serial_properties", serialroot);
        }
        else if(udppointer != NULL){
            udp_properties properties_ = udppointer->properties_;
            pt::ptree udproot;
            udproot.put("host", properties_.host);
            udproot.put("hostport", properties_.hostport);
            udproot.put("bindport", properties_.bindport);
            udproot.put("udp_type", properties_.udp_type);

            thislinkroot.add_child("udp_properties", udproot);
        }
        pt::ptree linkoptionsroot;
        linkoptionsroot.put("link_name", info_.link_name);
        linkoptionsroot.put("sim_enable", info_.sim_enable);
        linkoptionsroot.put("sim_packet_loss", info_.sim_packet_loss);
        linkoptionsroot.put("output_to", info_.output_to);
        linkoptionsroot.put("receive_from", info_.receive_from);
        linkoptionsroot.put("reject_repeat_packets", info_.reject_repeat_packets);
        linkoptionsroot.put("sik_radio", info_.SiK_radio);
        thislinkroot.add_child("link_options",linkoptionsroot);
        linksroot.add_child(std::to_string(link_id_), thislinkroot);
    }

    jsonroot.add_child("links",linksroot);
    std::stringstream ss;

    pt::json_parser::write_json(ss,jsonroot);

    return ss.str();
}

void JsonApi::deleteLink(int link_id)
{
    
}

void JsonApi::addLink(int link_id)
{
    
}
