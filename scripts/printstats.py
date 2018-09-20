#!/usr/bin/python
import json
import pprint
import requests
import time

while True:
    r = requests.get('http://localhost:8000/stats')
    rjson = r.json()
    for stat in rjson["stats"]:
        if stat["name"] == "rfd":
            pprint.pprint(float(stat["drate_rx"]))

    time.sleep(0.1)

