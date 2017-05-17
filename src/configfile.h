#ifndef CONFIG_FILE_H__
#define CONFIG_FILE_H__

#include <string>
#include <map>
#include <vector>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "mlink.h"
#include "serial.h"
#include "asyncsocket.h"

class ConfigFile
{
    std::map   <std::string, std::string> content_;
    std::vector<std::string>              sections_;
public:
    ConfigFile();
    ConfigFile(std::string const& configFile);

    std::vector<std::string> GetSections();

    // These functions are used to retrieve config file values
    // values are returned by reference
    // the bool return value signifies whether the requested value was successfully found parsed
    bool boolValue(std::string const& section, std::string const& entry, bool* value);
    bool intValue(std::string const& section, std::string const& entry, int* value);
    bool strValue(std::string const& section, std::string const& entry, std::string* value);
};

void readLinkInfo(ConfigFile* _configFile, std::string thisSection, LinkOptions* _info);
int readConfigFile(std::string &filename, std::vector<std::shared_ptr<mlink> > &links);

enum UDP_type {UDP_TYPE_NONE, UDP_TYPE_FULLY_SPECIFIED, UDP_TYPE_SERVER, UDP_TYPE_CLIENT, UDP_TYPE_BROADCAST};

#endif
