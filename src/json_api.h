/* CMAVNode
 * Monash UAS
 *
 * JSON API layer
 */

#ifndef JSON_API_H
#define JSON_API_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "linkmanager.h"
#include "mlink.h"
#include "serial.h"
#include "asyncsocket.h"

class JsonApi
{
public:
    JsonApi(std::shared_ptr<LinkManager> manager);
    ~JsonApi();

    std::string getLinks();

    bool removeLink(int link_id);
    void addLink(std::string json);

private:
    std::shared_ptr<LinkManager> manager_;

};
#endif
