#ifndef LINKMANAGER_H
#define LINKMANAGER_H

#include <boost/lockfree/spsc_queue.hpp>
#include <chrono>
#include <mutex>

#include "mlink.h"
#include "asyncsocket.h"
#include "serial.h"

#define Q_LINKS_TO_ADD_SIZE 10
#define Q_LINKS_TO_REMOVE_SIZE 10

#define CACHE_UPDATE_MS 50

typedef std::unordered_map<int,std::shared_ptr<mlink>> links_t;
typedef std::unordered_map<int,std::shared_ptr<MlinkCached>> links_cached_t;

class LinkManager
{
public:
    LinkManager(links_t *links, std::mutex &links_access_lock)
        :links_(links), links_access_lock_(links_access_lock) {};
    ~LinkManager() {};

    void updateLinksCache();

    links_cached_t getLinks() const;

    // These functions will be called from the JSON server
    // They return the link id of the created link, or -1 if failed
    int addSerial(serial_properties properties, link_options options);
    int addUDP(udp_properties properties, link_options options);

    bool removeLink(int link_id);

private:
    links_t *links_;

    // links_cached has infrequent read/write.
    // Cache is updated on every change
    mutable std::mutex links_cache_access_lock_;
    links_cached_t links_cached_;

    std::mutex &links_access_lock_;

    int newLinkID();

    int link_id_counter = 0;
};

#endif
