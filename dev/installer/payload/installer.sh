#!/bin/bash
#V A add in chmod scratch_gpio.sh
#Version B – go back to installing deb gpio package directly
#Version C – install sb files into /home/pi/Documents/Scratch Projects
#Version D – create “/home/pi/Documents/Scratch Projects” if it doesn’t exist
#Version E – change permissions on “/home/pi/Documents/Scratch Projects”
#Version F 13Oct12 – rem out rpi.gpio as now included in Raspbian
#Version G 20Mar13 - Allow otheruser option on commandline (Tnever/meltwater)
#Version H 24Mar13 - correct newline issues
f_exit(){
echo ""
echo "Usage:"
echo "i.e. sudo install_scratch_gpio.sh otheruser"
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
#echo "Is the above Home directory and User/Group correct (1/2)?"
#select yn in "Continue" "Cancel"; do
#    case $yn in
#        Continue ) break;;
#        Cancel ) f_exit;;
#    esac
#done


mkdir -p $HDIR/simplesi_scratch_handler

cp scratch_gpio_handler2.py $HDIR/simplesi_scratch_handler

#Instead of copying the scratch_gpio2.sh file, we will generate it
#Create a new file for scratch_gpio2.sh
echo "#!/bin/bash" > $HDIR/simplesi_scratch_handler/scratch_gpio2.sh
echo "#Version 0.2 - add in & to allow simulatenous running of handler and Scratch" >> $HDIR/simplesi_scratch_handler/scratch_gpio2.sh
echo "#Version 0.3 - change sp launches rsc.sb from \"/home/pi/Documents/Scratch Projects\"" >> $HDIR/simplesi_scratch_handler/scratch_gpio2.sh
echo "#Version 0.4 - 20Mar13 meltwater - change to use provided name for home" >> $HDIR/simplesi_scratch_handler/scratch_gpio2.sh
echo "sudo ps aux | grep 'python.*scratch_gpio_handler2.py' | grep -v grep | awk '{print \$2}' | xargs sudo kill -9 " >> $HDIR/simplesi_scratch_handler/scratch_gpio2.sh
echo "sudo python $HDIR/simplesi_scratch_handler/scratch_gpio_handler2.py &" >> $HDIR/simplesi_scratch_handler/scratch_gpio2.sh
echo "scratch --document \"$HDIR/Documents/Scratch Projects/rsc.sb\" &" >> $HDIR/simplesi_scratch_handler/scratch_gpio2.sh

sudo chmod +x $HDIR/simplesi_scratch_handler/scratch_gpio2.sh

cp blink11.py $HDIR

cp scratchgpio2.desktop $HDIR/Desktop

mkdir -p $HDIR/Documents/Scratch\ Projects
sudo chown -R $USERID:$GROUPID $HDIR/Documents/Scratch\ Projects
cp rsc.sb $HDIR/Documents/Scratch\ Projects
cp GPIOexample.sb $HDIR/Documents/Scratch\ Projects
cp blink11.sb $HDIR/Documents/Scratch\ Projects
echo ""
echo "Finished."
