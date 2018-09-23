#!/usr/bin/python
import json
import pprint
import requests
import sys
import time

filterlink = "";
filter = False

if len(sys.argv) > 1:
    filter = True
    filterlink = sys.argv[1]
    print sys.argv[1]

    print "Only displaying link " + filterlink
    time.sleep(1);

while True:
    r = requests.get('http://localhost:8000/stats')
    rjson = r.json()
    for stat in rjson["stats"]:
        sysstring = ": "
        noise = stat["local_noise"]
        rssi = stat["local_rssi"]
        for sysid in stat["sysids"]:
            sysstring = sysstring + sysid + " "

        if stat["name"] == filterlink or not filter:
            print stat["name"] + ": " + str(round(float(stat["drate_rx"]),2)) + " bps   S/N: " + str(rssi) + "/" + str(noise) + "   sysid" + sysstring
        #if stat["name"] == "SITL":
        #    pprint.pprint(float(stat["drate_rx"]))

    time.sleep(0.1)

