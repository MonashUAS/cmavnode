# cmavnode
MAVLink forwarding node written in C++

This program can forward packets between an arbitrary number of MAVLink connections.
Supports UDP, and Serial.

## Installing

- Clone the repository

- Update Git submodules 

         git submodule update --init

- Install dependencies

         Ubuntu 14.04: sudo apt-get install libboost-all-dev cmake libconfig++ libreadline-dev
         Ubuntu 16.04: sudo apt-get install libboost-all-dev cmake libconfig++-dev libreadline-dev
         Debian Stretch: sudo apt-get install libboost-all-dev cmake libconfig++-dev libreadline-dev
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

Use -i to get an interactive shell, type help into the shell to list commands.

## Licence
Cmavnode is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

Cmavnode is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Cmavnode. If not, see http://www.gnu.org/licenses/.
