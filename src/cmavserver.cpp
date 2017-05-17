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
            thislinkroot.put("sim_enable", info_->sim_enable);
            thislinkroot.put("sim_packet_loss", info_->sim_packet_loss);
            thislinkroot.put("output_to", info_->output_to);
            thislinkroot.put("receive_from", info_->receive_from);
            thislinkroot.put("reject_repeat_packets", info_->reject_repeat_packets);
            thislinkroot.put("sik_radio", info_->SiK_radio);
            linksroot.add_child(info_->link_name, thislinkroot);
        }

        jsonroot.add_child("links",linksroot);
        std::stringstream ss;

        pt::json_parser::write_json(ss,jsonroot);

        // I think using tellp here is dangerous but i'm not sure why... reconsider
        *response << "HTTP/1.1 200 OK\r\nContent-Length:" << ss.tellp() << "\r\n\r\n" << ss.str();
    };

    sServer->resource["^/tragedy$"]["GET"]=[](std::shared_ptr<HttpServer::Response> response, std::shared_ptr<HttpServer::Request> request) {

        std::string responsestr = "Did you ever hear the tragedy of Darth Plagueis The Wise? I thought not. It’s not a story the Jedi would tell you. It’s a Sith legend. Darth Plagueis was a Dark Lord of the Sith, so powerful and so wise he could use the Force to influence the midichlorians to create life… He had such a knowledge of the dark side that he could even keep the ones he cared about from dying. The dark side of the Force is a pathway to many abilities some consider to be unnatural. He became so powerful… the only thing he was afraid of was losing his power, which eventually, of course, he did. Unfortunately, he taught his apprentice everything he knew, then his apprentice killed him in his sleep. Ironic. He could save others from death, but not himself.";

        *response << "HTTP/1.1 200 OK\r\nContent-Length:" << responsestr.length() << "\r\n\r\n" << responsestr;
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
