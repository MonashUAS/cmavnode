#ifndef SHELL_H
#define SHELL_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include "mlink.h"
#include <readline/readline.h>
#include <readline/history.h>
void runShell(bool &exitMainLoop, std::vector<std::shared_ptr<mlink> > &links);
void executeLine(char *line, bool &exitMainLoop,
                 std::vector<std::shared_ptr<mlink> > &links);
void printLinkStats(std::vector<std::shared_ptr<mlink> > *links);
int findlink(std::string link_string, std::shared_ptr<mlink>* prt,
             std::vector<std::shared_ptr<mlink> > &links);
void printLinkQuality(std::vector<std::shared_ptr<mlink> > *links);

extern std::vector<std::shared_ptr<mlink>> links;
#endif
