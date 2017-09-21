#include "cmavserver.h"

namespace pt = boost::property_tree;

CmavServer::CmavServer(int serverport, LinkManager &manager)
{
    // store a pointer to the links structure
    manager_ = &manager;

    Address addr(Ipv4::any(), serverport);

    endpoint_ = std::make_shared<Http::Endpoint>(addr);

    initServer();

    server_thread = boost::thread(&CmavServer::serverThread, this);
    std::cout << "HTTP Server running on port " << serverport << std::endl;
}

CmavServer::~CmavServer()
{
    endpoint_->shutdown();
    server_thread.join();
    std::cout << "cmavserver detructed" << std::endl;
}


void CmavServer::serverThread(){
    std::cout << "HTTP Server thread started" << std::endl;
    start();
}

void CmavServer::setupRoutes()
{
    using namespace Rest;
    Routes::Get(router_, "/links", Routes::bind(&CmavServer::getLinks, this));
    Routes::Get(router_, "/links/:value", Routes::bind(&CmavServer::getLinkById, this));
}

void CmavServer::initServer()
{
    auto opts = Http::Endpoint::options()
        .threads(1);
    endpoint_->init(opts);

    setupRoutes();
}

void CmavServer::start()
{
    endpoint_->setHandler(router_.handler());
    endpoint_->serveThreaded();
}

void CmavServer::getLinks(const Rest::Request& request, Http::ResponseWriter response)
{
    response.send(Http::Code::Ok, std::string("seems to work"));
}

void CmavServer::getLinkById(const Rest::Request& request, Http::ResponseWriter response)
{
    int value = 0;
    if (request.hasParam(":value")) {
        value = request.param(":value").as<int>();
    }
    std::stringstream ss;
    ss << "Got a request to get link " << value;

    response.send(Http::Code::Ok, ss.str());
}
