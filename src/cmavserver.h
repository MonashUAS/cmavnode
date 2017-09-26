#ifndef CMAVSERVER_H
#define CMAVSERVER_H

#include <memory>
#include <boost/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional/optional.hpp>

#include <pistache/http.h>
#include <pistache/router.h>
#include <pistache/endpoint.h>

#include "mlink.h"
#include "serial.h"
#include "asyncsocket.h"
#include "linkmanager.h"
#include "json_api.h"

class CmavServer
{
public:
    CmavServer(int serverport, std::shared_ptr<JsonApi> json_api);
    ~CmavServer();

    void addHandlers();
    void serverThread();

    void setupRoutes();
    void initServer();
    void start();

    void getLinks(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void getLinkById(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);

    void addLink(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);

private:

    std::shared_ptr<Pistache::Http::Endpoint> endpoint_;
    Pistache::Rest::Router router_;
    boost::thread server_thread;

    // This assumes that the links vector wont get deallocated while the http server is running... is this safe?
    std::vector<std::shared_ptr<mlink>> *links;

    std::shared_ptr<JsonApi> json_api_;

};

#endif
