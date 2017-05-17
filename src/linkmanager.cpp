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
    return false;
}

void LinkManager::operate()
{
    std::cout << "LinkManager Operating" << std::endl;
}
