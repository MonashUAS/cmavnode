#ifndef CMAVSERVER_H
#define CMAVSERVER_H

#include "mlink.h"
#include "serial.h"
#include "asyncsocket.h"
#include "linkmanager.h"

#include <memory>
#include <boost/thread.hpp>
#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional/optional.hpp>

#include <pistache/http.h>
#include <pistache/router.h>
#include <pistache/endpoint.h>

using namespace Pistache;

class CmavServer
{
 public:
    CmavServer(int serverport, LinkManager &manager);
    ~CmavServer();

    void addHandlers();
    void serverThread();

    void setupRoutes();
    void initServer();
    void start();

    void getLinks(const Rest::Request& request, Http::ResponseWriter response);
    void getLinkById(const Rest::Request& request, Http::ResponseWriter response);

 private:

    std::shared_ptr<Http::Endpoint> endpoint_;
    Rest::Router router_;
    boost::thread server_thread;

    // This assumes that the links vector wont get deallocated while the http server is running... is this safe?
    std::vector<std::shared_ptr<mlink>> *links;

    LinkManager *manager_;

};

#endif
