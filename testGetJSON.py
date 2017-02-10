#!/usr/bin/env python
        
from sgh_GetJSONFromURL import GetJSONFromURL
        
j=GetJSONFromURL()
wappid = "4041655e60abaea9a9134b6e78ca864f"
wcitycountry = "Chorley,uk"
                                
msg = j.getJSON("http://api.openweathermap.org/data/2.5/weather?q=" + wcitycountry + ",&appid=" + wappid)
loop = 1
for x in msg.items():
    print loop , x
    loop += 1