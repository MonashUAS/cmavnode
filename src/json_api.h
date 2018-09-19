/* CMAVNode
 * Monash UAS
 *
 * JSON API layer
 */

#ifndef JSON_API_H
#define JSON_API_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include "linkmanager.h"
#include "mlink.h"
#include "serial.h"
#include "asyncsocket.h"
#include "routing.h"
#include "blockxmit.h"

class JsonApi
{
public:
 JsonApi(std::shared_ptr<LinkManager> manager,std::shared_ptr<blockXmit> blockxmit,source_map_t mapping,routing_table_t routing,std::mutex &links_access_lock) : manager_(manager),block_xmit_(blockxmit), mapping_(mapping), routing_(routing), links_access_lock_(links_access_lock) {};
    ~JsonApi() {};

    std::string getLinks() const;
    std::string getStats() const;
    std::string getMapping() const;
    std::string getRouting() const;

    void parseFile(std::string filename);
    bool removeLink(int link_id);
    void addLink(std::string json);
    void setMapping(std::string json);
    void setRouting(std::string json);
    bool sendFile(std::string json);

private:
    std::shared_ptr<LinkManager> manager_;
    std::shared_ptr<blockXmit> block_xmit_;
    source_map_t mapping_;
    routing_table_t routing_;
    std::mutex &links_access_lock_;
};
#endif
