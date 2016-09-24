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
void printLinkStats(std::vector<std::unique_ptr<mlink>> *links);

extern std::vector<std::unique_ptr<mlink>> links;
extern bool exitMainLoop;
#endif
