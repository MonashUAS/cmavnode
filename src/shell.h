#ifndef SHELL_H
#define SHELL_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include "mlink.h"
#include <readline/readline.h>
#include <readline/history.h>
void runShell(void);
void executeLine(char * line);
void printLinkStats(std::vector<std::shared_ptr<mlink>> *links);
int findlink(std::string link_string, std::shared_ptr<mlink>* prt);

extern std::vector<std::shared_ptr<mlink>> links;
extern bool exitMainLoop;
#endif
