/* CMAVNode
 * Monash UAS
 *
 * JSON API layer
 */

#include "json_api.h"

namespace pt = boost::property_tree;

std::string JsonApi::getLinks() const
{
    links_cached_t cache = manager_->getLinks();
    pt::ptree jsonroot;
    pt::ptree linksroot;

    for(auto it : cache)
    {
        auto thislink = it.second;
        // TODO: factor out the conversion of properties and options into json
        int link_id_ = it.first;
        link_options info_ = thislink->link_options_;
        auto serialpointer = std::dynamic_pointer_cast<SerialCached>(thislink);
        auto udppointer = std::dynamic_pointer_cast<AsyncSocketCached>(thislink);

        pt::ptree thislinkroot;
        thislinkroot.put("id",link_id_);

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
        linksroot.push_back(std::make_pair("", thislinkroot));
    }

    jsonroot.add_child("links",linksroot);
    std::stringstream ss;

    pt::json_parser::write_json(ss,jsonroot);

    return ss.str();
}

bool JsonApi::removeLink(int link_id)
{
    // Nothing to do at the json layer, just pass through
    return manager_->removeLink(link_id);
}

void JsonApi::addLink(std::string json)
{
    pt::ptree pt;
    std::stringstream ss;
    ss << json;
    read_json(ss, pt);

    // TODO: use link id to serve location back to client
    int created_link_id;
    link_options options;

    boost::optional< pt::ptree& > up_root = pt.get_child_optional("udp_properties");
    boost::optional< pt::ptree& > sp_root = pt.get_child_optional("serial_properties");
    boost::optional< pt::ptree& > lo_root = pt.get_child_optional("link_options");

    if(lo_root)
    {
        std::cout << "json_api adding link" << std::endl;
        pt::ptree lo_root_raw = lo_root.get();
        options.link_name =             lo_root_raw.get<std::string>("link_name");
        //options.sim_enable =            lo_root_raw.get<bool>("sim_enable");
        //options.sim_packet_loss =       lo_root_raw.get<int>("sim_packet_loss");
        //options.output_to =             lo_root_raw.get<bool>("output_to");
        //options.receive_from =          lo_root_raw.get<bool>("receive_from");
        //options.reject_repeat_packets = lo_root_raw.get<bool>("reject_repeat_packets");
        //options.SiK_radio =             lo_root_raw.get<bool>("sik_radio");
        options.output_only_from.push_back(0);
    }
    else
    {
        std::cout << "no link options" << std::endl;
        throw std::exception();
    }

    if(up_root)
    {
        std::cout << "json_api adding udp" << std::endl;
        pt::ptree up_root_raw = up_root.get();
        udp_properties properties;
        properties.udp_type = up_root_raw.get<int>("udp_type");
        properties.host =     up_root_raw.get<std::string>("host");
        properties.hostport = up_root_raw.get<int>("hostport");
        properties.bindport = up_root_raw.get<int>("bindport");

        created_link_id = manager_->addUDP(properties,options);
    }
    else if(sp_root)
    {
        std::cout << "json_api adding serial" << std::endl;
        pt::ptree sp_root_raw = sp_root.get();
        serial_properties properties;
        properties.port = sp_root_raw.get<std::string>("port");
        properties.baudrate = sp_root_raw.get<int>("baudrate");
        properties.flowcontrol = sp_root_raw.get<bool>("flowcontrol");

        created_link_id = manager_->addSerial(properties,options);
    }
    else
    {
        std::cout << "Not serial or udp what are you doing?" << std::endl;
        throw std::exception();
    }
}
