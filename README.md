# cmavnode
MAVLink forwarding node written in C++

This program can forward packets between an arbitrary number of MAVLink connections.
Supports UDP, and Serial is being worked on.

## Installing

- Clone the repository

- Update Git submodules 

         git submodule update --init

- Install dependencies
                           
         Arch Linux: sudo pacman -S boost cmake libconfig
         Ubuntu: sudo apt-get install libboost-all-dev cmake libconfig++
* Build cmavnode

         mkdir build && cd build
         cmake ..
         make
- OPTIONAL: Install cmavnode to /opt/cmavnode and add to path:
         sudo make install
         echo "alias cmavnode='/opt/cmavnode/cmavnode" >> ~/.bashrc
- OPTIONAL: Build cmavnode for debuging:
         (All messages will be printed to stdout and logged)
         mkdir debug && cd debug
         cmake .. -DCMAKE_BUILD_TYPE=Debug
         make

## Usage

    ./cmavnode --file configfilepath

cmavnode is configured by a config file. This config file specifies the links you want (socket or serial) and allows you to dictate routing rules. Please see example.cfg for how this works.

## Routing Logic

cmavnode listens for heartbeats on all links. Based on heartbeats received, it maintains internal routing tables to forward mavlink packets to the correct link.

Packets that have no target system, or have a target system set to 0 or -1, are broadcast messages. Broadcast messages are forwarded on all links which ***do not*** point to the system the packet came from.

Packets that have a target system field will be forwarded on all links which point to the target system.
