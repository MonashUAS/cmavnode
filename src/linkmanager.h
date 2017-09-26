#ifndef LINKMANAGER_H
#define LINKMANAGER_H

#include <boost/lockfree/spsc_queue.hpp>
#include <chrono>

#include "mlink.h"
#include "asyncsocket.h"
#include "serial.h"

#define Q_LINKS_TO_ADD_SIZE 10
#define Q_LINKS_TO_REMOVE_SIZE 10

#define CACHE_UPDATE_MS 50

class LinkManager
{
public:
    LinkManager(std::vector<std::shared_ptr<mlink>> *links_);
    ~LinkManager();

    // check if LinkManager needs to do anything with the vector
    bool hasPending();

    bool shouldUpdateCache();

    void updateCache();

    // Make any necessary changes to the links vector
    void operate();

    // These functions will be called from the JSON server
    // They return the link id of the created link, or -1 if failed
    int addSerial(serial_properties properties, LinkOptions options);
    int addUDP(udp_properties properties, LinkOptions options);

    bool removeLink(int link_id);

    bool editLink();

private:
    boost::lockfree::spsc_queue<std::shared_ptr<mlink>> q_links_to_add {Q_LINKS_TO_ADD_SIZE};
    boost::lockfree::spsc_queue<int> q_links_to_remove {Q_LINKS_TO_REMOVE_SIZE};

    std::vector<std::shared_ptr<mlink>> *links;

    std::vector<std::shared_ptr<MlinkCached>> links_cached_;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_cache_update_;

    int newLinkID();

    int link_id_counter = 0;
};

#endif
