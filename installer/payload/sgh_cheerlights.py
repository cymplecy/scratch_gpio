#!/usr/bin/env python
import urllib2
import json


class CheerLights():
    def __init__(self):
        self.lastID = 0
        self.urlRoot = "http://api.thingspeak.com/channels/1417/"
        self.colours = []

    # retrieve and load the JSON data into a JSON object
    def getJSON(self, url):
        try:
            jsonFeed = urllib2.urlopen(self.urlRoot + url,timeout = 3)
            feedData = jsonFeed.read()
            # print feedData
            jsonFeed.close()
            data = json.loads(feedData)
            # data = feedData
            return data
        except:
            print(" ")
            print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ")
            print("Error of somesort in getJSON for cheerlights")

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
