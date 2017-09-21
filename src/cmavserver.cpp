#include "cmavserver.h"

namespace pt = boost::property_tree;

typedef SimpleWeb::Server<SimpleWeb::HTTP> HttpServer;

CmavServer::CmavServer(int serverport, LinkManager &manager, std::vector<std::shared_ptr<mlink>> *links_arg)
{
    // store a pointer to the links structure
    links = links_arg;
    manager_ = &manager;
    //construct server
    sServer = std::make_shared<HttpServer>();
    sServer->config.port=serverport;


    // add handlers to the resource mapping
    addHandlers();

    server_thread = boost::thread(&CmavServer::serverThread, this);
    std::cout << "HTTP Server running" << std::endl;
}

void CmavServer::addHandlers()
{
    sServer->resource["^/links$"]["GET"]=[this](std::shared_ptr<HttpServer::Response> response, std::shared_ptr<HttpServer::Request> request) {

        pt::ptree jsonroot;
        pt::ptree linksroot;

        for(int i = 0; i < links->size(); i++)
        {
            // TODO: factor out the conversion of properties and options into json
            int link_id_ = (links->at(i))->getLinkID();
            LinkOptions *info_ = &(links->at(i)->info);
            std::shared_ptr<serial> serialpointer = std::dynamic_pointer_cast<serial>(links->at(i));
            std::shared_ptr<asyncsocket> udppointer = std::dynamic_pointer_cast<asyncsocket>(links->at(i));
            pt::ptree thislinkroot;
            if(serialpointer != NULL){
                serial_properties *properties_ = &(serialpointer->properties);
                pt::ptree serialroot;
                serialroot.put("port", properties_->port);
                serialroot.put("baudrate", properties_->baudrate);
                serialroot.put("flowcontrol", properties_->flowcontrol);

                thislinkroot.add_child("serial_properties", serialroot);
            }
            else if(udppointer != NULL){
                udp_properties *properties_ = &(udppointer->properties);
                pt::ptree udproot;
                udproot.put("host", properties_->host);
                udproot.put("hostport", properties_->hostport);
                udproot.put("bindport", properties_->bindport);
                udproot.put("udp_type", properties_->udp_type);

                thislinkroot.add_child("udp_properties", udproot);
            }
            pt::ptree linkoptionsroot;
            linkoptionsroot.put("link_name", info_->link_name);
            linkoptionsroot.put("sim_enable", info_->sim_enable);
            linkoptionsroot.put("sim_packet_loss", info_->sim_packet_loss);
            linkoptionsroot.put("output_to", info_->output_to);
            linkoptionsroot.put("receive_from", info_->receive_from);
            linkoptionsroot.put("reject_repeat_packets", info_->reject_repeat_packets);
            linkoptionsroot.put("sik_radio", info_->SiK_radio);
            thislinkroot.add_child("link_options",linkoptionsroot);
            linksroot.add_child(std::to_string(link_id_), thislinkroot);
        }

        jsonroot.add_child("links",linksroot);
        std::stringstream ss;

        pt::json_parser::write_json(ss,jsonroot);

        // I think using tellp here is dangerous but i'm not sure why... reconsider
        *response << "HTTP/1.1 200 OK\r\n" << "Access-Control-Allow-Origin: http://127.0.0.1\r\n" << "Content-Length:" << ss.tellp() << "\r\n\r\n" << ss.str();
    };

    sServer->resource["^/links$"]["POST"] = [this](std::shared_ptr<HttpServer::Response> response, std::shared_ptr<HttpServer::Request> request) {
        try {
            pt::ptree pt;
            read_json(request->content, pt);


            std::cout << "Got a link create post: " << std::endl;

            int created_link_id;
            LinkOptions options;
            boost::optional< pt::ptree& > up_root = pt.get_child_optional("udp_properties");
            boost::optional< pt::ptree& > sp_root = pt.get_child_optional("serial_properties");
            boost::optional< pt::ptree& > lo_root = pt.get_child_optional("link_options");
            if(lo_root)
            {
                pt::ptree lo_root_raw = lo_root.get();
                options.link_name =             lo_root_raw.get<std::string>("link_name");
                options.sim_enable =            lo_root_raw.get<bool>("sim_enable");
                options.sim_packet_loss =       lo_root_raw.get<int>("sim_packet_loss");
                options.output_to =             lo_root_raw.get<bool>("output_to");
                options.receive_from =          lo_root_raw.get<bool>("receive_from");
                options.reject_repeat_packets = lo_root_raw.get<bool>("reject_repeat_packets");
                options.SiK_radio =             lo_root_raw.get<bool>("sik_radio");
                options.output_only_from.push_back(0);
            }
            else
            {
                std::cout << "no link options" << std::endl;
                throw std::exception();
            }
            if(up_root)
            {
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

            pt::ptree retroot;
            retroot.add_child(std::to_string(created_link_id), pt);
            std::stringstream ss;
            boost::property_tree::json_parser::write_json(ss, retroot);
            std::string outputstring = ss.str();

            *response << "HTTP/1.1 200 OK\r\n"
            << "Content-Length: " << outputstring.length() << "\r\n\r\n"
            << outputstring;
        }
        catch(const std::exception &e) {
            *response << "HTTP/1.1 400 Bad Request\r\nContent-Length: " << strlen(e.what()) << "\r\n\r\n"
            << e.what();
        }
    };

    sServer->resource["^/links/([0-9]+)$"]["DELETE"] = [this](std::shared_ptr<HttpServer::Response> response, std::shared_ptr<HttpServer::Request> request) {
        try {
            std::cout << "delete req" << std::endl;
            std::string number = request->path_match[1];

            bool success = manager_->removeLink(std::stoi(number));

            *response << "HTTP/1.1 204 OK\r\n"
            << "Access-Control-Allow-Origin: http://127.0.0.1\r\n\r\n";
        }
        catch(const std::exception &e) {
            *response << "HTTP/1.1 400 Bad Request\r\nContent-Length: " << strlen(e.what()) << "\r\n\r\n"
            << e.what();
        }
    };
    sServer->default_resource["OPTIONS"] = [this](std::shared_ptr<HttpServer::Response> response, std::shared_ptr<HttpServer::Request> request) {
        *response << "HTTP/1.1 200 OK\r\n"
        << "Access-Control-Allow-Origin: http://127.0.0.1\r\n"
        << "Access-Control-Allow-Methods: DELETE\r\n"
        << "Content-Length: " << 0 << "\r\n\r\n";

    };
}

void CmavServer::serverThread(){
    sServer->start();
}

CmavServer::~CmavServer()
{
    std::cout << "cmavserver detructing" << std::endl;
    sServer->stop();
    server_thread.join();
}
