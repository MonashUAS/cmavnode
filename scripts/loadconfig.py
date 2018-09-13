#!/usr/bin/python
import json
import pprint
import time
from codecs import BOM_UTF8
import requests
import sys

def lstrip_bom(str_, bom=BOM_UTF8):
    if str_.startswith(bom):
        return str_[len(bom):]
    else:
        return str_


with open(sys.argv[1] , 'r') as f:
    data = f.read()
    final = json.loads(lstrip_bom(data))
    links = final["links"]
    mapping = final["mapping"]
    routing_table = final["routing_table"]

    for link in links:
        print 'Loading Link: ' + link['link_options']['link_name']
        packedlink = json.dumps(link)
        response = requests.post('http://localhost:8000/links', data=packedlink)

    time.sleep(0.1)

    print 'loading mapping'
    packedmapping = json.dumps(mapping)
    response = requests.post('http://localhost:8000/mapping', data=packedmapping)

    time.sleep(0.1)

    print 'loading routing table'
    packedtable = json.dumps(routing_table)
    response = requests.post('http://localhost:8000/routing', data=packedtable)
