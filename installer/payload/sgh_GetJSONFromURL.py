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
                ret.update(self.parse_dict(val, key + '_'))
            elif isinstance(val, list):
                for listitem in val:
                    ret.update(self.parse_dict({key + str(val.index(listitem)) : listitem}))
            else:
                ret[key] = val
        return ret

    # retrieve and load the JSON data into a JSON object
    
    def getJSON(self, url):
        #jdata = '{"coord":{"lon":-2.62,"lat":53.65},"weather":[{"id":300,"main":"Drizzle","description":"light intensity drizzle","icon":"09n"},{"id":741,"main":"Fog","description":"fog","icon":"50n"}],"base":"stations","main":{"temp":277.14,"pressure":1009,"humidity":100,"temp_min":276.15,"temp_max":278.15},"visibility":10000,"wind":{"speed":5.1,"deg":160},"clouds":{"all":40},"dt":1486450200,"sys":{"type":1,"id":5098,"message":0.0029,"country":"GB","sunrise":1486453471,"sunset":1486487131},"id":2653086,"name":"Chorley","cod":200} '
        
        #feedData = jdata
        jsonFeed = urllib2.urlopen(url)
        feedData = jsonFeed.read()
        print
        print feedData
        print
        #jsonFeed.close()
        data = json.loads(feedData)
        #return self.parse_dict(data)
        return self.parse_dict(data)
        
        
        
#j=GetJSONFromURL()

#msg = j.getJSON("test")
#loop = 1
#for x in msg.items():
#    print loop , x
#    loop += 1