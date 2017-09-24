/* CMAVNode
 * Monash UAS
 *
 * JSON API layer
 */

#ifndef JSON_API_H
#define JSON_API_H

#include <boost/property_tree/ptree.hpp>

class JsonApi
{
public:
    JsonApi();
    ~JsonApi();

    boost::property_tree::ptree getLinks();
    boost::property_tree::ptree getLink(int link_id);

    void deleteLink(int link_id);

    void addLink(int link_id)

}
#endif
