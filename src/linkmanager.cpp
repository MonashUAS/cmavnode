#include "linkmanager.h"

void LinkManager::updateLinksCache()
{
    // obtain lock on links_cached_
    std::lock_guard<std::mutex> lock(links_cache_access_lock_);

    links_cached_.clear();

    for(const auto it : *links_)
    {
        auto serialcheck = std::dynamic_pointer_cast<serial>(it.second);
        auto udpcheck = std::dynamic_pointer_cast<asyncsocket>(it.second);

        if(serialcheck)
        {
            std::shared_ptr<SerialCached> serial_cached = std::shared_ptr<SerialCached>(new SerialCached);
            serial_cached->properties_ = serialcheck->properties;
            serial_cached->link_id_ = serialcheck->getLinkID();
            serial_cached->link_options_ = serialcheck->info;
            links_cached_[serial_cached->link_id_] = (std::dynamic_pointer_cast<MlinkCached>(serial_cached));
        }
        else if(udpcheck)
        {
            std::shared_ptr<AsyncSocketCached> udp_cached = std::shared_ptr<AsyncSocketCached>(new AsyncSocketCached);
            udp_cached->properties_ = udpcheck->properties;
            udp_cached->link_id_ = udpcheck->getLinkID();
            udp_cached->link_options_ = udpcheck->info;
            links_cached_[udp_cached->link_id_] = (std::dynamic_pointer_cast<MlinkCached>(udp_cached));
        }
    }
}

links_cached_t LinkManager::getLinks() const
{
    // obtain threadsafe copy of pointers
    std::lock_guard<std::mutex> lock(links_cache_access_lock_);
    return links_cached_;
}

int LinkManager::lookupLinkByName(std::string name)
{
  std::lock_guard<std::mutex> lock(links_cache_access_lock_);
  //look through the cached map to find the matching link
  for (const auto & [ id, l ] : links_cached_) {
    if(l->link_options_.link_name.compare(name) == 0)
      return id;
  }
  return -1;
}

int LinkManager::addSerial(serial_properties properties, link_options options)
{
    std::lock_guard<std::mutex> lock(links_access_lock_);

    std::cout << "LinkManager: Creating Serial Link" << std::endl;
    int link_id_ = newLinkID();
    (*links_)[link_id_] = (std::shared_ptr<mlink>(new serial(properties,link_id_,options)));

    updateLinksCache();
    return link_id_;
}

int LinkManager::addUDP(udp_properties properties,  link_options options)
{
    std::lock_guard<std::mutex> lock(links_access_lock_);

    std::cout << "LinkManager: Creating UDP Link" << std::endl;
    int link_id_ = newLinkID();
    (*links_)[link_id_] = (std::shared_ptr<mlink>(new asyncsocket(properties,link_id_,options)));

    updateLinksCache();
    return link_id_;
}

bool LinkManager::removeLink(int link_id)
{
    std::lock_guard<std::mutex> lock(links_access_lock_);

    std::cout << "LinkManager: Deleting Link " << link_id << std::endl;
    for(auto it : *links_)
    {
        if(it.first == link_id)
            {
                links_->erase(link_id);

                updateLinksCache();
                return true;
            }
    }

    return false;
}

int LinkManager::newLinkID()
{
    return link_id_counter++;
}
