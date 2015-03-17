import sgh_captouch



class sgh_captouch_helper():

    def __init__(self):
        self.ctTrigStatus = [[5,0],[6,0],[7,0],[8,0],[1,0],[2,0],[3,0],[4,0],[9,0]]
        self.ctobject = None

    def ctHandler(self,channel,event):
        #print "handler received an event:",channel,event
        if self.ctTrigStatus[channel][1] == 0:
            self.ctTrigStatus[channel][1] = 1 #set channel to tocuhed
            if self.ctTrigStatus[8][1] == 0:
                self.ctTrigStatus[8][1] = 1 #set gloabl touched flag
