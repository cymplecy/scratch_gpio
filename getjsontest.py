#!/usr/bin/env python

from sgh_GetJson import GetJsonFromURL

getjsonfromurl = GetJsonFromURL()

data = getjsonfromurl.getJSON("http://api.openweathermap.org/data/2.5/weather?q=Chorley,uk&appid=4041655e60abaea9a9134b6e78ca864f")

