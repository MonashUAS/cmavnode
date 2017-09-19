#include "cmavserver.h"

namespace pt = boost::property_tree;

typedef SimpleWeb::Server<SimpleWeb::HTTP> HttpServer;

CmavServer::CmavServer(int serverport, std::vector<std::shared_ptr<mlink>> *links_arg)
{
    // store a pointer to the links structure
    links = links_arg;
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
        *response << "HTTP/1.1 200 OK\r\nContent-Length:" << ss.tellp() << "\r\n\r\n" << ss.str();
    };

    sServer->resource["^/links/delete$"]["POST"] = [this](std::shared_ptr<HttpServer::Response> response, std::shared_ptr<HttpServer::Request> request) {
        try {
            pt::ptree pt;
            read_json(request->content, pt);

            auto name = pt.get<std::string>("linkname");

            std::cout << "Got a link delete post: " << name;

            *response << "HTTP/1.1 200 OK\r\n"
            << "Content-Length: " << name.length() << "\r\n\r\n"
            << name;
        }
        catch(const std::exception &e) {
            *response << "HTTP/1.1 400 Bad Request\r\nContent-Length: " << strlen(e.what()) << "\r\n\r\n"
            << e.what();
        }
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
