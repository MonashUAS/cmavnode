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
#include "routing.h"


class JsonApi
{
public:
    JsonApi(std::shared_ptr<LinkManager> manager,source_map_t mapping) : manager_(manager), mapping_(mapping) {};
    ~JsonApi() {};

    std::string getLinks() const;
    std::string getMapping() const;

    bool removeLink(int link_id);
    void addLink(std::string json);

private:
    std::shared_ptr<LinkManager> manager_;
    source_map_t mapping_;
};
#endif
