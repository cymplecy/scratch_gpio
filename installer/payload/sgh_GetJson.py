#!/usr/bin/env python
import urllib2
import json


class GetJSONFromURL():
    def __init__(self):
        self.lastID = 0

        
    def parse_dict(self,init, lkey=''):
        ret = {}
        for rkey,val in init.items():
            key = lkey+rkey
            if isinstance(val, dict):
                ret.update(self.parse_dict(val, key+'_'))
            else:
                ret[key] = val
        return ret

    # retrieve and load the JSON data into a JSON object
    def getJSON(self, url):
        jsonFeed = urllib2.urlopen(url)
        feedData = jsonFeed.read()
        #print feedData
        jsonFeed.close()
        data = json.loads(feedData)
        print
        #print  data.get("temp")
        print self.parse_dict(data)
        print
        return self.parse_dict(data)

          
