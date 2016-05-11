# cmavnode
MAVLink forwarding node written in C++

This program can forward packets between an arbitrary number of MAVLink connections.
Supports UDP, and Serial is being worked on.

## Installing

- Clone the repository

- Update Git submodules 

         git submodule update --init

- Install dependencies
                           
         sudo pacman -S boost
         sudo apt-get install libboost-all-dev
* Build cmavnode

         mkdir bin && make

## Usage

    bin/cmavnode --socket=<targetip>:<targetport:listenport --serial=<port>:<baudrate>

For example, typical usage on a companion computer would be to have a serial port connecting to the autopilot, a socket forwarding packets to the ground, and a socket forwarding packets to other companion computer software. (e.g. dronekit) This would be done as follows

    bin/cmavnode --socket=192.168.1.10:14550:14555 --socket=0.0.0.0:14551:14552 --serial=/dev/ttyAMA0:57600

## Routing Logic

cmavnode listens for heartbeats on all links. Based on heartbeats received, it maintains internal routing tables to forward mavlink packets to the correct link.

Packets that have no target system, or have a target system set to 0 or -1, are broadcast messages. Broadcast messages are forwarded on all links which ***do not*** point to the system the packet came from.

Packets that have a target system field will be forwarded on all links which point to the target system.
