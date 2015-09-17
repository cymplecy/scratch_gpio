#!/usr/bin/env python
# sgh_RasPiCamera - control the raspberry pi camera via scratch
#Copyright (C) 2015 by Matt Venn

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

Version = '0.0.2'  # 15 Sep 15 Fixed ownership of directory

import os
import glob
import pwd
import grp


class RasPiCamera:

    def __init__(self):
        print "pi camera init"
        self.num = 0

        self.user = os.getenv("SUDO_USER")
        # get uid/gid so we can chown the photo
        self.uid = pwd.getpwnam(self.user).pw_uid
        self.gid = grp.getgrnam(self.user).gr_gid
        self.dir = ("/home/" + self.user + "/photos/")

        try:
            os.mkdir(self.dir)
            os.chown(self.dir, self.uid, self.gid)
        except OSError:
            # already exists
            pass

        photos = glob.glob(self.dir + '*.jpg')
        # if there are existing photos, find the highest numbered
        if len(photos):
            photos = [x.replace(self.dir, '') for x in photos]
            photos = [x.replace('.jpg', '') for x in photos]
            photos = [int(x) for x in photos]
            latest_photo_num = sorted(photos)[-1]
            self.num = latest_photo_num + 1
        print("starting at %d" % self.num)

    def take_photo(self):

        photo_file = self.dir + str(self.num) + '.jpg'
        # try with raspistill for the RasPi camera board
        os.system("raspistill -n -t 1 -o " + photo_file)

        # try with fswebcam for usb devices
        if not os.path.isfile(photo_file):
            os.system("fswebcam -r 1024x768 " + photo_file)

        # change ownership if we have a photo
        if os.path.isfile(photo_file):
            os.chown(photo_file, self.uid, self.gid)
            print "photo taken: " + photo_file
            self.num += 1
        # otherwise, error message
        else:
            print("Error taking photo - camera probably not correctly fitted")
            print("if using a USB webcam, ensure fswebcam is installed")

#### end RasPiCamera ###############################################################