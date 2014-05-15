#!/usr/bin/env python
# sgh_RasPiCamera - control the raspberry pi camera via scratch
#Copyright (C) 2014 by Matt Venn

#This program is free software; you can redistribute it and/or
#modify it under the terms of the GNU General Public License
#as published by the Free Software Foundation; either version 2
#of the License, or (at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

#You should have received a copy of the GNU General Public License
#along with this program; if not, write to the Free Software
#Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

Version =  '0.0.1sw3' # 12May14 Investigate photo folder problem using SID


import os

class RasPiCamera:

    def __init__(self):
        print "pi camera init"
        self.num = 0
        realUser = os.getenv("SUDO_USER")
        print "user",realUser
        self.dir = ("/home/"+ realUser + "/photos/")
        try:
            os.mkdir(self.dir)
        except OSError:
            #already exists
            pass
        print "complete"


    def take_photo(self):
        photo_file = self.dir + str(self.num) + '.jpg'
        os.system("raspistill -o " + photo_file)
        print "photo taken: " + photo_file
        self.num += 1

#### end RasPiCamera ###############################################################




