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
		std::cout << "\theart <link>\t\tlist heartbeat count for the link." <<std::endl;
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
    else if(!linestring.compare(0,5,"packet"))
    {
        if(linestring.size() >= 7)
        {
            std::string link_to_do = linestring.substr(6,std::string::npos);

            std::shared_ptr<mlink> linkfound;

            if(findlink(link_to_do, &linkfound, links))
            {

                linkfound->printPacketStats();
            }
            else std::cout << "Link " << link_to_do << " not found" << std::endl;

        }
        else
        {
            std::cout << "heart requires parameters" << std::endl;
        }

    }
}

int findlink(std::string link_string, std::shared_ptr<mlink>* prt,
             std::vector<std::shared_ptr<mlink> > &links)
{

    int numberlink;
    bool isnumber = true;
    bool found = false;
    try
    {
        numberlink = stoi(link_string);
    }
    catch(std::invalid_argument& e)
    {

        isnumber = false;
    }
    catch(std::out_of_range& e)
    {
        isnumber = false;
    }

    if(isnumber) // this is bad it assumes less than 10 links
    {
        for(int i = 0; i < links.size(); i++)
        {
            if(numberlink == links.at(i)->link_id)
            {
                *prt = links.at(i);
                return 1;
            }
        }
    }
    else //not number
    {

        for(int i = 0; i < links.size(); i++)
        {
            if(!link_string.compare(links.at(i)->info.link_name))
            {
                *prt = links.at(i);
                return 1;
            }
        }


    }
    return 0;
}
