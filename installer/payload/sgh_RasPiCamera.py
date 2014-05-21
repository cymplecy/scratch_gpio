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

Version =  '0.0.1sw4' # 21May14 Merge pull request from Matt Venn

import os,glob,pwd,grp

class RasPiCamera:

    def __init__(self):
        print "pi camera init"
        self.num = 0

        self.user = os.getenv("SUDO_USER")
        self.dir = ("/home/"+ self.user + "/photos/")

        try:
            os.mkdir(self.dir)
        except OSError:
            #already exists
            pass

        photos = glob.glob(self.dir + '*.jpg')
        if len(photos):
            latest_photo = sorted(photos)[-1]
            latest_photo = latest_photo.replace(self.dir,'')
            latest_photo = latest_photo.replace('.jpg','')
            latest_photo_num = int(latest_photo)
            self.num = latest_photo_num + 1

        print("starting at %d" % self.num)


    def take_photo(self):
        photo_file = self.dir + str(self.num) + '.jpg'
        os.system("raspistill -n -t 1 -o " + photo_file)
        print "photo taken: " + photo_file

	#chown the photo so it can be modified by the user
	uid = pwd.getpwnam(self.user).pw_uid
	#assume group is same name as user
	gid = grp.getgrnam(self.user).gr_gid
	os.chown(photo_file, uid, gid)
        self.num += 1

#### end RasPiCamera ###############################################################




