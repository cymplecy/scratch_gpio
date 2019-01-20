#!/bin/bash


# A very crude script to setup samba inspired by MrEngmans Realtek RTL8188CUS script
#  V1.1 - added in apt-get update
#  V1.2 - public share (e.g / files) can now be edited using remote pi user
#  V1.3 - server name now automatically taken from /etc/hostname 20Jan16
#  V1.4 - root files can now be edited again using remote pi user 16Feb16
#  V1.5 - updated as errors with lastest Raspbain 15Map16
#  V1.5a 9Jul16 - add force-yes to apt-get update

echo
echo "This script will install Samba (windows networking) in a very simple manner"
echo " "
echo "It only requires you to choose a password"
echo "I recommend just using raspberry :)"
echo " "
read -p "Press any key to continue..." -n1 -s
echo
echo

#	while true; do
#		echo
#		read -p "Please enter the name your wish to give your RaspberryPi - " RPINAME
#		echo
#		echo "You have named your RaspberryPi as \"$RPINAME\", is that correct?"
#		read -p "press Y to continue, any other key to re-enter the name. " -n1 RESPONSE
#		if [ "$RESPONSE" == "Y" ] || [ "$RESPONSE" == "y" ]; then
#			echo
#			break
#		fi
#		echo
#	done
apt-get update -y --force-yes
apt-get install --force-yes samba
apt-get install --force-yes samba-common-bin
smbpasswd -a pi
echo "#======================= Global Settings =======================" > /etc/samba/smb.conf
echo "[global]" >> /etc/samba/smb.conf
echo "workgroup = WORKGROUP" >> /etc/samba/smb.conf
echo "wide links = yes" >> /etc/samba/smb.conf
echo "unix extensions = no" >> /etc/samba/smb.conf


#echo ";server string = " $RPINAME " server" >> /etc/samba/smb.conf
#echo ";netbios name = " $RPINAME >> /etc/samba/smb.conf

echo "dns proxy = no" >> /etc/samba/smb.conf

echo "#### Debugging/Accounting ####" >> /etc/samba/smb.conf
echo "log file = /var/log/samba/log.%m" >> /etc/samba/smb.conf
echo "max log size = 1000" >> /etc/samba/smb.conf
echo "syslog = 0" >> /etc/samba/smb.conf
echo "panic action = /usr/share/samba/panic-action %d" >> /etc/samba/smb.conf

echo "####### Authentication #######" >> /etc/samba/smb.conf
echo "security = user" >> /etc/samba/smb.conf
echo "map to guest = Bad User" >> /etc/samba/smb.conf
echo "guest account = pi" >> /etc/samba/smb.conf

echo "#======================= Share Definitions =======================" >> /etc/samba/smb.conf

echo "[root]" >> /etc/samba/smb.conf
echo "comment = Admin Config Share" >> /etc/samba/smb.conf
echo "path = /" >> /etc/samba/smb.conf
echo "browseable = yes" >> /etc/samba/smb.conf
echo "force user = root" >> /etc/samba/smb.conf
echo "force group = root" >> /etc/samba/smb.conf
echo "admin users = pi" >> /etc/samba/smb.conf
echo "writeable = yes" >> /etc/samba/smb.conf
echo "read only = no" >> /etc/samba/smb.conf
echo "guest ok = yes" >> /etc/samba/smb.conf
echo "create mask = 0777" >> /etc/samba/smb.conf
echo "directory mask = 0777" >> /etc/samba/smb.conf

echo "#-------------------------------------------------------------------" >> /etc/samba/smb.conf

echo "[pi]" >> /etc/samba/smb.conf
echo "comment = pi user /homepi folder" >> /etc/samba/smb.conf
echo "path = /home/pi" >> /etc/samba/smb.conf
echo "browseable = yes" >> /etc/samba/smb.conf
echo "force user = pi" >> /etc/samba/smb.conf
echo "force group = pi" >> /etc/samba/smb.conf
echo "admin users = pi" >> /etc/samba/smb.conf
echo "writeable = yes" >> /etc/samba/smb.conf
echo "read only = no" >> /etc/samba/smb.conf
echo "guest ok = yes" >> /etc/samba/smb.conf
echo "create mask = 0777" >> /etc/samba/smb.conf
echo "directory mask = 0777" >> /etc/samba/smb.conf


sudo samba restart


# time to finish!

echo
echo 
#echo "Have fun with " $RPINAME"
echo
echo "Remember to logon as user=pi password= 'the password you have chosen' your windows machines"
echo
