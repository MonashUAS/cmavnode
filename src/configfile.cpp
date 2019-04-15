#include "configfile.h"

#include <fstream>
#include "../include/mavlink2/mavlink_get_info.h"

int readConfigFile(std::string &filename, std::vector<std::shared_ptr<mlink> > &links)
{
    ConfigFile _configFile = ConfigFile(filename);

    std::vector<std::string> sections = _configFile.GetSections();
    std::cout << "Found " << sections.size() << " links" << std::endl;

    for (uint i = 0; i < sections.size(); i++)
    {
        std::string thisSection = sections.at(i);
        std::string type;
        bool isSerial = false;
        UDP_type udp_type_ = UDP_TYPE_NONE;
        if(!_configFile.strValue(thisSection, "type", &type))
        {
            std::cerr << "Link has no type - skipping" << std::endl;
            continue;
        }
        std::string serialport;
        int baud;
        std::string targetip;
        std::string bindip;
        std::string bcastip;
        bool flowcontrol = false;
        bool bcastlock = true;
        int targetport = 0;
        int localport = 0;
        int bcastport = 0;

        if( type.compare("serial") == 0)
        {
            if(!_configFile.strValue(thisSection, "port",&serialport) || !_configFile.intValue(thisSection, "baud", &baud))
            {
                std::cerr << "Link: " << thisSection << " is specified as serial but does not have valid port and baud" << std::endl;
                continue;
            }

            //Try to get flow control
            _configFile.boolValue(thisSection, "flow_control", &flowcontrol);
            isSerial = true;
            std::cout << "Valid Serial Link: " << thisSection << " Found at: " << serialport << ", baud: " << baud << std::endl;
        }
        else if(type.compare("udp") == 0 || type.compare("socket") == 0 || type.compare("udpbcast") == 0)
        {
            if(type.compare("udpbcast") == 0 && _configFile.intValue(thisSection, "bcastport", &bcastport) && _configFile.strValue(thisSection, "bcastip", &bcastip))
            {
                if(!_configFile.strValue(thisSection, "bindip", &bindip))
                {
                    // setting bindip in config file specifies interface to broadcast on
                    bindip = "0.0.0.0"; //If bind ip not specified use 0.0.0.0... ipv4_any()
                }

                if(bcastip.find("255") == std::string::npos)
                {
                    std::cerr << "Link: " << thisSection << " does not have a valid broadcast address" << std::endl;
                    continue;
                }
                _configFile.boolValue(thisSection, "bcastlock", &bcastlock);
                udp_type_ = UDP_TYPE_BROADCAST;
            }
            else if(_configFile.strValue(thisSection, "targetip", &targetip)
                    && _configFile.intValue(thisSection, "targetport", &targetport)
                    && _configFile.intValue(thisSection, "localport", &localport))
            {
                udp_type_ = UDP_TYPE_FULLY_SPECIFIED;
            }
            else if(_configFile.intValue(thisSection, "localport", &localport))
            {
                udp_type_ = UDP_TYPE_SERVER;
            }
            else if(_configFile.strValue(thisSection, "targetip", &targetip)
                    && _configFile.intValue(thisSection, "targetport", &targetport))
            {
                udp_type_ = UDP_TYPE_CLIENT;
            }
            else if(_configFile.intValue(thisSection, "targetport", &targetport))
            {
                targetip = "localhost";
                udp_type_ = UDP_TYPE_CLIENT;
            }
            else
            {
                std::cerr << "Link: " << thisSection << " is specified as " << type << " but does not have valid ip and port" << std::endl;
                continue;
            }
            if(udp_type_ != UDP_TYPE_BROADCAST)
            {
                std::cout << "Valid UDP Link: " << thisSection << " Found at " << targetip << ":" << targetport << " -> " << localport << std::endl;
            }
            else
            {
                std::cout << "Valid UDPBroadcast Link: " << thisSection << " Found, broadcasting on port " << bcastport << " bound to: " << bindip << std::endl;
            }
        }
        else
        {
            std::cerr << "Link: " << thisSection << " has invalid link type: " << type << std::endl;
            continue;
        }

        link_info _info;
        readLinkInfo(&_configFile, thisSection, &_info);
        //if we made it this far without break we have a valid link of some sort
        if(isSerial)
        {
            links.push_back(std::shared_ptr<mlink>(new serial(serialport
                                                   ,std::to_string(baud)
                                                   ,flowcontrol
                                                   ,_info)));
        }
        else if (udp_type_ != UDP_TYPE_NONE)
        {
            switch(udp_type_)
            {
            case UDP_TYPE_FULLY_SPECIFIED:
                links.push_back(
                    std::shared_ptr<mlink>(new asyncsocket(targetip,
                                           std::to_string(targetport)
                                           ,std::to_string(localport)
                                           ,_info)));
                break;
            case UDP_TYPE_SERVER:
                links.push_back(
                    std::shared_ptr<mlink>(new asyncsocket(std::to_string(localport)
                                           ,_info)));
                break;
            case UDP_TYPE_CLIENT:
                links.push_back(
                    std::shared_ptr<mlink>(new asyncsocket(targetip,
                                           std::to_string(targetport)
                                           ,_info)));
                break;
            case UDP_TYPE_BROADCAST:
                links.push_back(
                    std::shared_ptr<mlink>(new asyncsocket(bcastlock,
                                           bindip,
                                           bcastip,
                                           std::to_string(bcastport)
                                           ,_info)));
                break;
            }
        }
    }
    return 0;
}

void readLinkInfo(ConfigFile* _configFile, std::string thisSection, link_info* _info)
{
    // Parse the optional parts of the config file which end up in mlink::link_info
    std::vector<int> output_only_from;
    std::string output_list_tmp;
    if(_configFile->strValue(thisSection, "output_only_from",&output_list_tmp ))
    {
        std::vector< std::string> output_only_from_strings;
        boost::split(output_only_from_strings, output_list_tmp, boost::is_any_of(","));
        for(unsigned int i = 0; i < output_only_from_strings.size(); i++)
        {
            //this is not typesafe need to fix
            output_only_from.push_back( atoi( output_only_from_strings.at(i).c_str() ) );
        }
    }
    else
    {
        output_only_from.push_back(0);
    }

    _info->link_name = thisSection;
    _info->output_only_from = output_only_from;
    _configFile->boolValue(thisSection, "sim_enable", &_info->sim_enable);
    if(_info->sim_enable)
    {
        //then sim_enable is true
        std::cout << "WARNING: Link has simulation options enabled" << std::endl;
        if(_configFile->intValue(thisSection, "sim_packet_loss", &_info->sim_packet_loss))
        {
            std::cout << "Packet loss set to " << _info->sim_packet_loss << "%" << std::endl;
        }
    }

    // Enable or disable packet dropping
    _configFile->boolValue(thisSection, "reject_repeat_packets", &_info->reject_repeat_packets);

    // Identify SiK radio links
    _configFile->boolValue(thisSection, "sik_radio", &_info->SiK_radio);

    // Enable sleep mode for the link
    _configFile->boolValue(thisSection, "sleep", &_info->sleep_enabled);

    std::string filter_string;
    if (_configFile->strValue(thisSection, "filter", &filter_string))
    {
        // Find the filter type separator
        size_t filter_type_separator = filter_string.find_first_of(':');

        // Filter type seporator has been found
        if (filter_type_separator != std::string::npos)
        {
            const std::unordered_map<std::string, link_filter_type> filter_type_map = 
            {
                { "DROP", link_filter_type::DROP },
                { "ACCEPT", link_filter_type::ACCEPT },
            };

            // Extract the filter type string
            std::string filter_type_str = filter_string.substr(0, filter_type_separator);

            // Find the binding in the map
            auto filter_type_iter = filter_type_map.find(filter_type_str);

            // It's a valid filter type
            if (filter_type_iter != filter_type_map.end())
            {
                // Extract the filter messages string
                std::string filter_messages_str = filter_string.substr(filter_type_separator + 1);

                std::vector<std::string> messages_strs;

                boost::split(messages_strs, filter_messages_str, boost::is_any_of(","));

                // Filter is not empty
                if (messages_strs[0].length())
                {
                    _info->filter_type = filter_type_iter->second;

                    const mavlink_message_info_t *message_info;

                    // For every message name
                    for (const std::string &filter_message_str : messages_strs) {
                        // Find the MAVLink message information
                        message_info = mavlink_get_message_info_by_name(filter_message_str.c_str());

                        // Valid message name
                        if (message_info)
                            _info->filter_messages.insert(message_info->msgid);
                        // Invalid message name
                        else
                        std::cout << "Failed to add message \"" << filter_message_str << "\" to the filter. Unknown message!"
                            << std::endl; 
                    } 
                }
                // Empty filter
                else
                    std::cout << "Failed to load filter for \"" << _info->link_name << "\". No messages!" << std::endl;
            }
            // Unknown filter type
            else 
                std::cout << "Failed to load filter for \"" << _info->link_name << "\". Uknown filter type \"" <<
                    filter_type_str << "\"!" << std::endl;
        }
        // No filter message type
        else
            std::cout << "Failed to load filter for \"" << _info->link_name << "\". No filter type found!" << std::endl;
    }
}

std::string trim(std::string const& source, char const* delims = " \t\r\n")
{
    std::string result(source);
    std::string::size_type index = result.find_last_not_of(delims);
    if(index != std::string::npos)
        result.erase(++index);

    index = result.find_first_not_of(delims);
    if(index != std::string::npos)
        result.erase(0, index);
    else
        result.erase();
    return result;
}

ConfigFile::ConfigFile()
{

}

ConfigFile::ConfigFile(std::string const& configFile)
{
    std::ifstream file(configFile.c_str());

    std::string line;
    std::string name;
    std::string value;
    std::string inSection;
    int posEqual;
    while (std::getline(file,line))
    {

        if (! line.length()) continue;

        if (line[0] == '#') continue;
        if (line[0] == ';') continue;

        if (line[0] == '[')
        {
            inSection=trim(line.substr(1,line.find(']')-1));
            sections_.push_back(inSection);
            continue;
        }

        posEqual=line.find('=');
        name  = trim(line.substr(0,posEqual));
        value = trim(line.substr(posEqual+1));

        content_[inSection+'/'+name]=value;
    }
}

std::vector<std::string> ConfigFile::GetSections()
{
    return sections_;
}

bool ConfigFile::boolValue(std::string const& section, std::string const& entry, bool* value)
{
    std::string str_value;
    if(!strValue(section, entry, &str_value)) return false;

    if(str_value.compare("true") == 0)
    {
        *value = true;
        return true;
    }
    else if(str_value.compare("false") == 0)
    {
        *value = false;
        return true;
    }
    else if(str_value.compare("1") == 0)
    {
        *value = true;
        return true;
    }
    else if(str_value.compare("0") == 0)
    {
        *value = false;
        return true;
    }

    return false;

}
bool ConfigFile::intValue(std::string const& section, std::string const& entry, int* value)
{
    std::string str_value;
    if(!strValue(section, entry, &str_value)) return false;

    try
    {
        *value = std::stoi(str_value);
    }
    catch(std:: exception e)
    {
        //converting to int caused a problem
        return false;
    }
    return true;
}

bool ConfigFile::strValue(std::string const& section, std::string const& entry, std::string* value)
{
    std::map<std::string, std::string>::const_iterator ci = content_.find(section + '/' + entry);

    if (ci == content_.end()) return false; //then the requested value does not exist

    *value = ci->second;
    return true;
}
