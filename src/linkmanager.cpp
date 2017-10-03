#include "linkmanager.h"

LinkManager::LinkManager(std::vector<std::shared_ptr<mlink>> *links_, std::mutex &links_access_lock)
    :links_access_lock_(links_access_lock)
{
    // store the pointer to the links struct
    links = links_;
    std::cout << "LinkManager constructing" << std::endl;

    // Initialize the cache update clock
    last_cache_update_ = std::chrono::high_resolution_clock::now();
}

LinkManager::~LinkManager()
{
    std::cout << "LinkManager Destructing" << std::endl;
}

bool LinkManager::shouldUpdateCache()
{
    auto t1 = std::chrono::high_resolution_clock::now();
    auto int_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - last_cache_update_);
    if(int_ms.count() > CACHE_UPDATE_MS)
    {
        last_cache_update_ = t1;
        return true;
    }
    return false;
}

void LinkManager::updateCache()
{
    if(!shouldUpdateCache())
        return;

    // obtain lock on links_cached_
    std::lock_guard<std::mutex> lock(cache_access_lock_);

    links_cached_.clear();

    for(auto iter = links->begin(); iter < links->end(); iter++)
    {
        auto serialcheck = std::dynamic_pointer_cast<serial>(*iter);
        auto udpcheck = std::dynamic_pointer_cast<asyncsocket>(*iter);

        if(serialcheck)
        {
            std::shared_ptr<SerialCached> serial_cached = std::shared_ptr<SerialCached>(new SerialCached);
            serial_cached->properties_ = serialcheck->properties;
            serial_cached->link_id_ = serialcheck->getLinkID();
            serial_cached->link_options_ = serialcheck->info;
            serial_cached->link_quality_ = serialcheck->link_quality;
            links_cached_.push_back(std::dynamic_pointer_cast<MlinkCached>(serial_cached));
        }
        else if(udpcheck)
        {
            std::shared_ptr<AsyncSocketCached> udp_cached = std::shared_ptr<AsyncSocketCached>(new AsyncSocketCached);
            udp_cached->properties_ = udpcheck->properties;
            udp_cached->link_id_ = udpcheck->getLinkID();
            udp_cached->link_options_ = udpcheck->info;
            udp_cached->link_quality_ = udpcheck->link_quality;
            links_cached_.push_back(std::dynamic_pointer_cast<MlinkCached>(udp_cached));
        }
    }
}

std::vector<std::shared_ptr<MlinkCached>> LinkManager::getLinks()
{
    // obtain threadsafe copy of pointers
    std::lock_guard<std::mutex> lock(cache_access_lock_);
    return links_cached_;
}

int LinkManager::addSerial(serial_properties properties, LinkOptions options)
{
    std::cout << "LinkManager: Creating Serial Link" << std::endl;

    std::lock_guard<std::mutex> lock(links_access_lock_);

    int link_id_ = newLinkID();
    links->push_back(std::shared_ptr<mlink>(new serial(properties,link_id_,options)));

    updateCache();
    return link_id_;
}

int LinkManager::addUDP(udp_properties properties, LinkOptions options)
{
    std::cout << "LinkManager: Creating UDP Link" << std::endl;

    std::lock_guard<std::mutex> lock(links_access_lock_);
    std::cout << "LinkManager: Got lock on links" << std::endl;

    int link_id_ = newLinkID();
    links->push_back(std::shared_ptr<mlink>(new asyncsocket(properties,link_id_,options)));

    updateCache();
    return link_id_;
}

bool LinkManager::removeLink(int link_id)
{
    std::cout << "LinkManager: Deleting Link " << link_id << std::endl;

    std::lock_guard<std::mutex> lock(links_access_lock_);

    for(auto iter = links->begin(); iter < links->end(); iter++)
    {
        if((*iter)->getLinkID() == link_id)
            {
                links->erase(iter);

                updateCache();
                return true;
            }
    }

    return false;
}

int LinkManager::newLinkID()
{
    return link_id_counter++;
}
