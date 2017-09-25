/* CMAVNode
 * Monash UAS
 *
 * JSON API layer
 */

#ifndef JSON_API_H
#define JSON_API_H

#include <boost/property_tree/ptree.hpp>

#include "linkmanager.h"

class JsonApi
{
public:
    JsonApi(std::shared_ptr<LinkManager> manager);
    ~JsonApi();

    boost::property_tree::ptree getLinks();
    boost::property_tree::ptree getLink(int link_id);

    void deleteLink(int link_id);
    void addLink(int link_id);

private:
    std::shared_ptr<LinkManager> manager_;

};
#endif
