#ifndef LINKMANAGER_H
#define LINKMANAGER_H

#include <boost/lockfree/spsc_queue.hpp>

#include "mlink.h"
#include "asyncsocket.h"
#include "serial.h"

#define Q_LINKS_TO_ADD_SIZE 10
#define Q_LINKS_TO_REMOVE_SIZE 10

class LinkManager
{
 public:
    LinkManager(std::vector<std::shared_ptr<mlink>> *links_);
    ~LinkManager();

    // check if LinkManager needs to do anything with the vector
    bool hasPending();

    // Make any necessary changes to the links vector
    void operate();

    // These functions will be called from the JSON server
    bool addSerial(LinkOptions options, serial_properties properties);
    bool addUDP(LinkOptions options, udp_properties properties);

    bool removeLink();

    bool editLink();

  private:
    boost::lockfree::spsc_queue<std::shared_ptr<mlink>> links_to_add {Q_LINKS_TO_ADD_SIZE};
    boost::lockfree::spsc_queue<std::string> links_to_remove {Q_LINKS_TO_REMOVE_SIZE};

    std::vector<std::shared_ptr<mlink>> *links;
};

#endif
