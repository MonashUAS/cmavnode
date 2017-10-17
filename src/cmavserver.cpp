#include "cmavserver.h"

namespace pt = boost::property_tree;
using namespace Pistache;

CmavServer::CmavServer(int serverport, std::shared_ptr<JsonApi> json_api)
{
    json_api_ = json_api;

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


void CmavServer::serverThread()
{
    std::cout << "HTTP Server thread started" << std::endl;
    start();
}

void CmavServer::setupRoutes()
{
    using namespace Rest;
    Routes::Get(router_, "/links", Routes::bind(&CmavServer::getLinks, this));
    Routes::Get(router_, "/links/:value", Routes::bind(&CmavServer::getLinkById, this));


    Routes::Post(router_, "/links", Routes::bind(&CmavServer::addLink, this));

    Routes::Get(router_,"/heartbeat", Routes::bind(&CmavServer::handleHeartbeat, this));
    Routes::Options(router_,"/links/:value", Routes::bind(&CmavServer::respondOptions, this));
    Routes::Options(router_,"/links", Routes::bind(&CmavServer::respondOptions, this));
    Routes::Delete(router_, "/links/:value", Routes::bind(&CmavServer::removeLink, this));
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

void CmavServer::handleHeartbeat(const Rest::Request& request, Http::ResponseWriter response)
{
    addCors(response);
    response.send(Http::Code::No_Content);
}

void CmavServer::respondOptions(const Rest::Request& request, Http::ResponseWriter response)
{
    addCors(response);
    response.headers().add<Http::Header::AccessControlAllowMethods>(std::vector<Http::Method>{Http::Method::Delete, Http::Method::Post});
    response.headers().add<Http::Header::Allow>(std::vector<Http::Method>{Http::Method::Delete, Http::Method::Post});
    response.headers().add<Http::Header::ContentType>(MIME(Application,Json));
    response.headers().add<Http::Header::AccessControlAllowHeaders>("Content-Type");
    response.send(Http::Code::Ok);
}

void CmavServer::addLink(const Rest::Request& request, Http::ResponseWriter response)
{
    std::cout << "Cmavserver got an add" << std::endl;

    json_api_->addLink(request.body());

    addCors(response);
    response.send(Http::Code::Created);
}

void CmavServer::addCors(Http::ResponseWriter& response)
{
    response.headers().add<Http::Header::AccessControlAllowOrigin>("*");
}

void CmavServer::removeLink(const Rest::Request& request, Http::ResponseWriter response)
{
    int value = 0;
    if (request.hasParam(":value"))
    {
        value = request.param(":value").as<int>();

        if(json_api_->removeLink(value))
        {
            std::cout << "Link Deleted" << std::endl;
            addCors(response);
            response.send(Http::Code::No_Content);
        }
        else
        {
            std::cout << "Could not delete link" << std::endl;
            addCors(response);
            response.send(Http::Code::Not_Found);
        }
    }
    else
    {
        addCors(response);
        response.send(Http::Code::Bad_Request);
    }
}

void CmavServer::getLinks(const Rest::Request& request, Http::ResponseWriter response)
{
    std::string linksstring = json_api_->getLinks();
    addCors(response);
    response.send(Http::Code::Ok, linksstring);
}

void CmavServer::getLinkById(const Rest::Request& request, Http::ResponseWriter response)
{
    int value = 0;
    if (request.hasParam(":value"))
    {
        value = request.param(":value").as<int>();
    }
    std::stringstream ss;
    ss << "Got a request to get link " << value;

    response.send(Http::Code::Ok, ss.str());
}
