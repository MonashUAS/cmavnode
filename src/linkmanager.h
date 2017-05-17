#ifndef LINKMANAGER_H
#define LINKMANAGER_H

#include <boost/lockfree/queue.hpp>

#include "mlink.h"
#include "asyncsocket.h"
#include "serial.h"

#define Q_LINKS_TO_ADD_SIZE 50

class LinkManager
{
 public:
    LinkManager(std::vector<std::shared_ptr<mlink>> *links_);
    ~LinkManager();

    // check if LinkManager needs to do anything with the vector
    bool hasPending();

    // Make any necessary changes to the links vector
    void operate();

 private:

    std::vector<std::shared_ptr<mlink>> *links;
};

#endif
