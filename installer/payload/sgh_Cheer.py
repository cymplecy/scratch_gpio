import urllib2
import json
import time

lastID = 0      #most recent entry_id
  
#main program


#check for new colour requests
while True:
    jsonFeed = urllib2.urlopen("http://api.thingspeak.com/channels/1417/" +"field/1/last.json")
    feedData = jsonFeed.read()
    #print feedData
    jsonFeed.close()
    data = json.loads(feedData)
    if int(data["entry_id"]) > lastID:   #Has this entry_id been processed before?
        print "new colour", data["field1"]   #add the colour to the head
        lastID = int(data["entry_id"])
    else:
        time.sleep(10)
