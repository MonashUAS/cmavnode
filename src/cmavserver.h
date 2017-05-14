#ifndef CMAVSERVER_H
#define CMAVSERVER_H

#include "../include/simple_server/server_http.hpp"
#include "mlink.h"

#include <memory>
#include <boost/thread.hpp>
#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

class CmavServer
{
 public:
    CmavServer(int serverport, std::vector<std::shared_ptr<mlink>> *links_arg);
    ~CmavServer();

    void addHandlers();
    void serverThread();

 private:
    std::shared_ptr<SimpleWeb::Server<SimpleWeb::HTTP>> sServer;
    boost::thread server_thread;

    // This assumes that the links vector wont get deallocated while the http server is running... is this safe?
    std::vector<std::shared_ptr<mlink>> *links;

};

#endif
