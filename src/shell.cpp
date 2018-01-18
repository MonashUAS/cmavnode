#include "shell.h"

void runShell(bool &exitMainLoop, std::vector<std::shared_ptr<mlink> > &links)
{
    while(!exitMainLoop)
    {
        char * line = readline("CMAV > ");
        if(!line) break;
        if(*line) add_history(line);

        executeLine(line, exitMainLoop, links);

        free(line);
    }
}


void executeLine(char *line, bool &exitMainLoop, std::vector<std::shared_ptr<mlink> > &links)
{

    std::string linestring(line);

    if(!linestring.compare("stat"))
        printLinkStats(&links);
    else if(!linestring.compare("linkquality"))
        printLinkQuality(&links);
    else if(!linestring.compare("quit"))
        exitMainLoop = true;
    else if(!linestring.compare("help"))
    {
        std::cout << "Supported commands:" <<std::endl;
        std::cout << "\tstat\t\t\tgive link stats and system ids on each line." <<std::endl;
        std::cout << "\tlinkquality\t\tgive link quality stats for each link." << std::endl;
        std::cout << "\tpacket <link>\t\tlist packet count for the link." <<std::endl;
        std::cout << "\tdown <link>\t\tstop sending on this link." <<std::endl;
        std::cout << "\tup <link>\t\tstart sending on this link." <<std::endl;
        std::cout << "\tquit" <<std::endl;
    }
    else if(!linestring.compare(0,4,"down"))
    {
        if(linestring.size() >= 6)
        {
            std::string link_to_do = linestring.substr(5,std::string::npos);

            std::shared_ptr<mlink> linkfound;

            if(findlink(link_to_do, &linkfound, links))
            {

                linkfound->up = false;
                std::cout << "Link " << link_to_do << " DOWN" << std::endl;
            }
            else std::cout << "Link " << link_to_do << " not found" << std::endl;

        }
        else
        {
            std::cout << "down requires parameters" << std::endl;
        }
    }
    else if(!linestring.compare(0,2,"up"))
    {
        if(linestring.size() >= 4)
        {
            std::string link_to_do = linestring.substr(3,std::string::npos);

            std::shared_ptr<mlink> linkfound;

            if(findlink(link_to_do, &linkfound, links))
            {

                linkfound->up = true;
                std::cout << "Link " << link_to_do << " UP" << std::endl;
            }
            else std::cout << "Link " << link_to_do << " not found" << std::endl;

        }
        else
        {
            std::cout << "up requires parameters" << std::endl;
        }

    }
    else if(!linestring.compare(0,6,"packet"))
    {
        if(linestring.size() >= 8)
        {
            std::string link_to_do = linestring.substr(7,std::string::npos);

            std::shared_ptr<mlink> linkfound;

            if(findlink(link_to_do, &linkfound, links))
            {

                linkfound->printPacketStats();
            }
            else std::cout << "Link " << link_to_do << " not found" << std::endl;

        }
        else
        {
            std::cout << "packet requires parameters (link number or name)" << std::endl;
        }

    }
}

int findlink(std::string link_string, std::shared_ptr<mlink>* prt,
             std::vector<std::shared_ptr<mlink> > &links)
{

    bool found = false;

    for(int i = 0; i < links.size(); i++)
    {
        if(!link_string.compare(links.at(i)->info.link_name))
        {
            *prt = links.at(i);
            return 1;
        }
    }


    return 0;
}

void printLinkStats(std::vector<std::shared_ptr<mlink> > *links)
{
    std::cout << "---------------------------------------------------------------" << std::endl;
    // Print stats for each known link
    for (auto curr_link = links->begin(); curr_link != links->end(); ++curr_link)
    {
        std::ostringstream buffer;

        buffer << "Link: " << (*curr_link)->info.link_name << " ";
        if ((*curr_link)->is_kill)
        {
            buffer << "DEAD ";
        }
        else if ((*curr_link)->up)
        {
            buffer << "UP ";
        }
        else
        {
            buffer << "DOWN ";
        }

        buffer << "Received: " << (*curr_link)->totalPacketCount << " "
               << "Recv_kB/s: " << (*curr_link)->datarate_recv << " "
               << "Sent: " << (*curr_link)->totalPacketSent << " "
               << "Systems on link: ";

        std::map<uint8_t, mlink::packet_stats>* sysID_map = &((*curr_link)->sysID_stats);

        for(auto iter = sysID_map->begin(); iter != sysID_map->end(); iter++)
        {
            buffer << (int)iter->first << " ";
        }

        buffer << "InQueue: " << (*curr_link)->in_counter.get();
        buffer << " OutQueue: " << (*curr_link)->out_counter.get();
        boost::asio::ip::udp::endpoint *ep = (*curr_link)->sender_endpoint();
        if (ep)
        {
            buffer << " Host: " << ep->address().to_string().c_str() << ":" << (int)ep->port();
        }

        std::cout << buffer.str() << std::endl;
    }
    std::cout << "---------------------------------------------------------------" << std::endl;
}

void printLinkQuality(std::vector<std::shared_ptr<mlink> > *links)
{
    // Create a line of link quality
    std::ostringstream buffer;
    for (auto curr_link = links->begin(); curr_link != links->end(); ++curr_link)
    {
        buffer << "\nLink: " << "   (" << (*curr_link)->info.link_name << ")\n";

        // Only print radio stats when the link is connected to a SiK radio
        if ((*curr_link)->info.SiK_radio)
        {
            buffer  << std::setw(17)
                    << "Link delay: "<< std::setw(5) << (*curr_link)->link_quality.link_delay << " s\n"
                    << std::setw(17)
                    << "Local RSSI: " << std::setw(5) << (*curr_link)->link_quality.local_rssi
                    << std::setw(23)
                    << "Remote RSSI: " << std::setw(5) << (*curr_link)->link_quality.remote_rssi << "\n"
                    << std::setw(17)
                    << "Local noise: " << std::setw(5) << (*curr_link)->link_quality.local_noise
                    << std::setw(23)
                    << "Remote noise: " << std::setw(5) << (*curr_link)->link_quality.remote_noise << "\n"
                    << std::setw(17)
                    << "RX errors: " << std::setw(5) << (*curr_link)->link_quality.rx_errors
                    << std::setw(23)
                    << "Corrected packets: " << std::setw(5) << (*curr_link)->link_quality.corrected_packets << "\n"
                    << std::setw(17)
                    << "TX buffer: " << std::setw(5) << (*curr_link)->link_quality.tx_buffer << "%\n\n";
        }
        if ((*curr_link)->sysID_stats.size() != 0)
            buffer << std::setw(15) <<"System ID"
                   << std::setw(19) <<"Packets Lost"
                   << std::setw(19) <<"Packets Dropped"
                   << std::setw(19) <<"Packet Loss %" << "\n";
        for (auto iter = (*curr_link)->sysID_stats.begin(); iter != (*curr_link)->sysID_stats.end(); ++iter)
        {
            if ((*curr_link)->info.SiK_radio && iter->first == 51)
            {
                buffer << std::setw(11) << "(SiK)"
                       << std::setw(4) << (int)iter->first;
            }
            else
            {
                buffer << std::setw(15) << (int)iter->first;
            }
            buffer << std::setw(19) << iter->second.packets_lost
                   << std::setw(19) << iter->second.packets_dropped
                   << std::setw(19) << iter->second.packet_loss_percent << "\n";

        }
    }
    std::cout << "\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
              << buffer.str()
              <<   "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

}

