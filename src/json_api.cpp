/* CMAVNode
 * Monash UAS
 *
 * JSON API layer
 */

#include "json_api.h"

namespace pt = boost::property_tree;

//parse an entire json file
void JsonApi::parseFile(std::string filename)
{
  std::ifstream inFile;
  inFile.open(filename);
  if (!inFile) {
    std::cout << "Unable to open file";
    exit(1); // terminate with error
  }
  pt::ptree root;
  pt::read_json(inFile, root);

  inFile.close();

  boost::optional< pt::ptree& > links_root = root.get_child_optional("links");
  boost::optional< pt::ptree& > mapping_root = root.get_child_optional("mapping");
  boost::optional< pt::ptree& > routing_root = root.get_child_optional("routing_table");

  if(links_root)
  {
    pt::ptree links_root_raw = links_root.get();
    BOOST_FOREACH(pt::ptree::value_type &v, links_root_raw)
    {
      std::stringstream ss;
      pt::write_json(ss,v.second);
      addLink(ss.str());
    }
  }

  if(mapping_root)
  {
      pt::ptree mapping_root_raw = mapping_root.get();
      std::stringstream ss;
      pt::write_json(ss,mapping_root_raw);
      setMapping(ss.str());
  }

  if(routing_root)
  {
      pt::ptree routing_root_raw = routing_root.get();
      std::stringstream ss;
      pt::write_json(ss,routing_root_raw);
      setRouting(ss.str());
  }
}

std::string JsonApi::getStats() const
{
  links_cached_t cache = manager_->getLinks();
  pt::ptree jsonroot;
  pt::ptree statsroot;

  for(auto it : cache)
  {
    auto thislink = it.second;
    int link_id_ = it.first;
    link_stats stats_ = thislink->stats_;

    pt::ptree thislinkroot;
    thislinkroot.put("id",link_id_);
    thislinkroot.put("name",thislink->link_options_.link_name);
    thislinkroot.put("drate_rx",stats_.drate_rx);
    thislinkroot.put("local_rssi",stats_.local_rssi);
    thislinkroot.put("remote_rssi",stats_.remote_rssi);

    statsroot.push_back(std::make_pair("", thislinkroot));
  }

    jsonroot.add_child("stats",statsroot);

    std::stringstream ss;

    pt::json_parser::write_json(ss,jsonroot);

    return ss.str();
}

std::string JsonApi::getMapping() const
{
    pt::ptree jsonroot;
    pt::ptree maproot;

    for(auto it : *mapping_)
    {
        pt::ptree thismaproot;
        thismaproot.put("src",it.src);
        thismaproot.put("dst",it.dest);
        thismaproot.put("bidir",it.bidir);

        maproot.push_back(std::make_pair("",thismaproot));
    }

    jsonroot.add_child("mapping",maproot);
    std::stringstream ss;

    pt::json_parser::write_json(ss,jsonroot);

    return ss.str();
}

std::string JsonApi::getRouting() const
{
  pt::ptree jsonroot;
  pt::ptree routeroot;

  for(auto it : *routing_)
    {
      pt::ptree thisrouteroot;
      thisrouteroot.put("dest",it.dest);
      thisrouteroot.put("next_hop",it.next_hop);

      routeroot.push_back(std::make_pair("",thisrouteroot));
    }

  jsonroot.add_child("routing",routeroot);
  std::stringstream ss;

  pt::json_parser::write_json(ss,jsonroot);

  return ss.str();
}

void JsonApi::setMapping(std::string json)
{
    pt::ptree pt;
    std::stringstream ss;
    ss << json;
    read_json(ss, pt);

    // obtain lock on the main loop
    std::lock_guard<std::mutex> lock(links_access_lock_);

    mapping_->clear(); //empty the mapping
    BOOST_FOREACH(pt::ptree::value_type &v, pt) {
        uint8_t src = v.second.get<uint8_t>("src");
        uint8_t dst = v.second.get<uint8_t>("dst");
        bool bidir = v.second.get<bool>("bidir");
        std::cout << "src: " << (int)src << " dst: " << (int)dst << " bidir: " << (int)bidir << std::endl;
        mapping_->push_back(sys_pair(src,dst,bidir));
    }
}

bool JsonApi::sendFile(std::string json)
{
  pt::ptree pt;
  std::stringstream ss;
  ss << json;
  read_json(ss, pt);

  std::string filename = pt.get<std::string>("filename");
  int x = pt.get<int>("x");
  int y = pt.get<int>("y");
  return block_xmit_->sendFile(filename,x,y);
}
void JsonApi::setRouting(std::string json)
{
  pt::ptree pt;
  std::stringstream ss;
  ss << json;
  read_json(ss, pt);

  // obtain lock on the main loop
  std::lock_guard<std::mutex> lock(links_access_lock_);

  routing_->clear(); //empty the mapping
  BOOST_FOREACH(pt::ptree::value_type &v, pt) {
    std::string nh = v.second.get<std::string>("next_hop");

    //Find out if string is a number
    std::string::const_iterator it = nh.begin();
    while (it != nh.end() && std::isdigit(*it)) ++it;
    bool isnumber = !nh.empty() && it == nh.end();

    // Now get numeric value of dest either by lookup or conversion
    int nh_num = 0; 
    if(isnumber)
    {
      std::stringstream nhstream(nh); 
      nhstream >> nh_num;
    }
    else
    {
      nh_num = manager_->lookupLinkByName(nh);
    }
    uint8_t dest = v.second.get<uint8_t>("dest");
    std::cout << "dest: " << (int)dest << " next_hop: " << (int)nh_num << std::endl;
    routing_->push_back(route(dest,nh_num));
  }
}

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
        linkoptionsroot.put("reject_repeat_packets", info_.reject_repeat_packets);
        linkoptionsroot.put("sik_radio", info_.SiK_radio);
        linkoptionsroot.put("blockXmitRx", info_.blockXmitRx);
        linkoptionsroot.put("blockXmitTx", info_.blockXmitTx);
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
        pt::ptree lo_root_raw = lo_root.get();
        options.link_name =             lo_root_raw.get<std::string>("link_name");
        options.blockXmitRx =             lo_root_raw.get<bool>("blockXmitRx");
        options.blockXmitTx =             lo_root_raw.get<bool>("blockXmitTx");
        options.reject_repeat_packets = lo_root_raw.get<bool>("reject_repeat_packets");
        options.SiK_radio =             lo_root_raw.get<bool>("sik_radio");
    }
    else
    {
        std::cout << "Error: No link options" << std::endl;
        throw std::exception();
    }

    if(up_root)
    {
        pt::ptree up_root_raw = up_root.get();
        udp_properties properties;

        // Check that all the needed params are there
        boost::optional<int> udp_type_opt = up_root_raw.get_optional<int>("udp_type");
        if(!udp_type_opt) // Always fatal
        {
            std::cout << "Error: No UDP Type" << std::endl;
            return;
        }
        else
            properties.udp_type = udp_type_opt.get();

        boost::optional<std::string> host_opt = up_root_raw.get_optional<std::string>("host");
        if(!host_opt && properties.udp_type == 0) // Fatal if fully defined
        {
            std::cout << "Error: No Host" << std::endl;
            return;
        }
        else if(!host_opt) // Host can be infered
            properties.host = std::string("127.0.0.1");
        else // Host has been specified
            properties.host = host_opt.get();

        boost::optional<int> hostport_opt = up_root_raw.get_optional<int>("hostport");
        if(!hostport_opt && properties.udp_type != 2)
        {
            std::cout << "Error: No HostPort" << std::endl;
            return;
        }
        else if(!hostport_opt) // Value Doesnt matter
            properties.hostport = -1;
        else //value has been specified
            properties.hostport = hostport_opt.get();

        boost::optional<int> bindport_opt = up_root_raw.get_optional<int>("bindport");
        if(!bindport_opt && properties.udp_type != 1)
            {
                std::cout << "Error: No BindPort" << std::endl;
                return;
            }
        else if(!bindport_opt) // Value Doesnt matter
            properties.bindport = -1;
        else //value has been specified
            properties.bindport = bindport_opt.get();

        created_link_id = manager_->addUDP(properties,options);
    }
    else if(sp_root)
    {
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
