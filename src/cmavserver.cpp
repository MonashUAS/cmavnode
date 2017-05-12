#include "cmavserver.h"

using namespace boost::property_tree;

typedef SimpleWeb::Server<SimpleWeb::HTTP> HttpServer;

CmavServer::CmavServer(int serverport)
{
    std::cout << "cmavserver constructing" << std::endl;

    //construct server
    sServer = std::make_shared<HttpServer>();
    sServer->config.port=serverport;

    sServer->resource["^/test$"]["GET"]=[](std::shared_ptr<HttpServer::Response> response, std::shared_ptr<HttpServer::Request> request) {


        std::string responsestr = "Did you ever hear the tragedy of Darth Plagueis The Wise? I thought not. It’s not a story the Jedi would tell you. It’s a Sith legend. Darth Plagueis was a Dark Lord of the Sith, so powerful and so wise he could use the Force to influence the midichlorians to create life… He had such a knowledge of the dark side that he could even keep the ones he cared about from dying. The dark side of the Force is a pathway to many abilities some consider to be unnatural. He became so powerful… the only thing he was afraid of was losing his power, which eventually, of course, he did. Unfortunately, he taught his apprentice everything he knew, then his apprentice killed him in his sleep. Ironic. He could save others from death, but not himself.";

        *response << "HTTP/1.1 200 OK\r\nContent-Length:" << responsestr.length() << "\r\n\r\n" << responsestr;
    };

    server_thread = boost::thread(&CmavServer::serverThread, this);
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
