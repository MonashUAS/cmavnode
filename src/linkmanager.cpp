#include "linkmanager.h"

LinkManager::LinkManager(std::vector<std::shared_ptr<mlink>> *links_)
{
    // store the pointer to the links struct
    links = links_;
    std::cout << "LinkManager constructing" << std::endl;
}

LinkManager::~LinkManager()
{
    std::cout << "LinkManager Destructing" << std::endl;
}

bool LinkManager::hasPending()
{
    if(links_to_add.read_available() != 0)
        return true;

    if(links_to_remove.read_available() != 0)
        return true;

    return false;
}

void LinkManager::operate()
{
    std::cout << "LinkManager Operating" << std::endl;
}

bool LinkManager::addSerial(LinkOptions options, serial_properties properties)
{
    std::cout << "LinkManager: Coming soon" << std::endl;
}

bool LinkManager::addUDP(LinkOptions options, udp_properties properties)
{
    std::cout << "LinkManager: Coming soon" << std::endl;
}

bool LinkManager::removeLink()
{
    std::cout << "LinkManager: Coming soon" << std::endl;
}
