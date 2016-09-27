# cmavnode
MAVLink forwarding node written in C++

This program can forward packets between an arbitrary number of MAVLink connections.
Supports UDP, and Serial.

## Installing

- Clone the repository

- Update Git submodules 

         git submodule update --init

- Install dependencies

         Ubuntu: sudo apt-get install libboost-all-dev cmake libconfig++ libreadline-dev
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

    ./cmavnode -f <pathtoconfigfile>

cmavnode is configured by a config file. This config file specifies the links you want (socket or serial) and allows you to dictate routing rules. Please see examples to see how this works.

Use -i to get an interactive shell, type help to list commands.
