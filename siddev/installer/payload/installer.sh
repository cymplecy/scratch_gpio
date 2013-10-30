#!/bin/bash
#V A add in chmod scratch_gpio.sh
#Version B – go back to installing deb gpio package directly
#Version C – install sb files into /home/pi/Documents/Scratch Projects
#Version D – create “/home/pi/Documents/Scratch Projects” if it doesn’t exist
#Version E – change permissions on “/home/pi/Documents/Scratch Projects”
#Version F 13Oct12 – rem out rpi.gpio as now included in Raspbian
#Version G 20Mar13 - Allow otheruser option on commandline (Tnever/meltwater)
#Version H 24Mar13 - correct newline issues
#Version 1.0 3Aug13 - modified for use to deploy Remote Scratch Interface Device code
f_exit(){
echo ""
echo "Usage:"
echo "i.e. sudo install_sid.sh otheruser"
echo "Optional: Add a non-default 'otheruser' username after the command (default is:pi)."
exit
}

echo "Running Installer"
if [ -z $1 ]
then
HDIR="/home/pi"
USERID="pi"
GROUPID="pi"
else
HDIR=/home/$1
USERID=`id -n -u $1`
GROUPID=`id -n -g $1`
fi

#Confirm if install should continue with default PI user or inform about commandline option.
echo ""
echo "Install Details:"
echo "Home Directory: "$HDIR
echo "User: "$USERID
echo "Group: "$USERID
echo ""
if [ ! -d "$HDIR" ]; then
    echo ""; echo "The home directory does not exist!";f_exit;
fi

#Not needed as directory should already exist from sgh install
mkdir -p $HDIR/sid

cp sid.py $HDIR/sid

#Instead of copying the sid.sh file, we will generate it
#Create a new file for sid.sh
echo "#!/bin/bash" > $HDIR/sid/sid.sh
echo "#Version 0.2 - add in & to allow simulatenous running of handler and Scratch" >> $HDIR/sid/sid.sh
echo "#Version 0.3 - change sp launches rsc.sb from \"/home/pi/Documents/Scratch Projects\"" >> $HDIR/sid/sid.sh
echo "#Version 0.4 - 20Mar13 meltwater - change to use provided name for home" >> $HDIR/sid/sid.sh
echo "#V0.5 re-used for launching sid.py" >> $HDIR/sid/sid.sh
echo "sudo ps aux | grep 'python.*sid.py' | grep -v grep | awk '{print $2}' | xargs sudo kill -9 " >> $HDIR/sid/sid.sh
echo "sudo python /home/pi/sid/sid.py" >> $HDIR/sid/sid.sh

sudo chmod 4775 $HDIR/sid/sid.sh

cp sid.desktop $HDIR/Desktop
cp sid.desktop $HDIR/.config/autostart


echo ""
echo ""
echo "Finished installing Scratch Interface Device software"
