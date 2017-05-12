#ifndef CMAVSERVER_H
#define CMAVSERVER_H

#include "../include/simple_server/server_http.hpp"

#include <memory>
#include <boost/thread.hpp>
#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

class CmavServer
{
 public:
    CmavServer(int serverport);
    ~CmavServer();

    void serverThread();

 private:
    std::shared_ptr<SimpleWeb::Server<SimpleWeb::HTTP>> sServer;
    boost::thread server_thread;
};

#endif
