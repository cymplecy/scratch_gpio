#!/usr/bin/env python
import urllib2
import json


class GetJsonFromURL():
    def __init__(self):
        self.lastID = 0
        self.rtnDatacolours = []
        
       
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
        print self.parse_dict(data).get("main_temp")
        print
        return data

    # read the last entry_id
    def getEntryID(self, feed):
        return int(feed["entry_id"])

    def get_colours(self):
        last = self.getJSON("field/1/last.json")
        if self.getEntryID(last) > self.lastID:   # Have processed this entry_id before?
            self.colours = []
            data = self.getJSON("feed.json")
            for feed in data["feeds"]:
                self.colours = [str(feed["field1"])] + self.colours
                self.lastID = self.getEntryID(feed)
        return self.colours            
