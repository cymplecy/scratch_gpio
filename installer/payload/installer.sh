#!/bin/bash
#V A add in chmod scratch_gpio.sh
#Version B – go back to installing deb gpio package directly
#Version C – install sb files into /home/pi/Documents/Scratch Projects
#Version D – create “/home/pi/Documents/Scratch Projects” if it doesn’t exist
#Version E – change permissions on “/home/pi/Documents/Scratch Projects”
#Version F 13Oct12 – rem out rpi.gpio as now included in Raspbian
#Version G 20Mar13 - Allow otheruser option on commandline (Tnever/meltwater)
#Version H 24Mar13 - correct newline issues
#Version 29Oct13 Add in Chown commands and extra Adafruit and servod files and alter gpio_scrath2.sh and bit of chmod +x make V3
#Version 21Nov13 Change for ScratchGPIO V4
f_exit(){
echo ""
echo "Usage:"
echo "i.e. sudo install_scratchgpio4.sh otheruser"
echo "Optional: Add a non-default 'otheruser' username after the command (default is:pi)."
exit
}
echo "Debug Info"

#echo $SUDO_USER

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

echo "Please wait a few seconds"
sudo pkill -f servod
sleep 1

echo "Thank you"
sudo rm -rf $HDIR/scratchgpio4

mkdir -p $HDIR/scratchgpio4
chown -R $USERID:$GROUPID $HDIR/scratchgpio4

cp scratchgpio_handler4.py $HDIR/scratchgpio4
cp Adafruit_I2C.py $HDIR/scratchgpio4
cp Adafruit_PWM_Servo_Driver.py $HDIR/scratchgpio4
cp sgh_GPIOController.py $HDIR/scratchgpio4
cp sgh_PCF8591P.py $HDIR/scratchgpio4
cp sgh_PiGlow.py $HDIR/scratchgpio4
cp sgh_Stepper.py $HDIR/scratchgpio4

cp sgh_servod $HDIR/scratchgpio4
cp killsgh.sh $HDIR/scratchgpio4

chown -R $USERID:$GROUPID $HDIR/scratchgpio4
chmod +x sgh_servod
chmod +x killsgh.sh


#Instead of copying the scratchgpio4.sh file, we will generate it
#Create a new file for scratchgpio4.sh
echo "#!/bin/bash" > $HDIR/scratchgpio4/scratchgpio4.sh
echo "#Version 0.2 - add in & to allow simulatenous running of handler and Scratch" >> $HDIR/scratchgpio4/scratchgpio4.sh
echo "#Version 0.3 - change sp launches rsc.sb from \"/home/pi/Documents/Scratch Projects\"" >> $HDIR/scratchgpio4/scratchgpio4.sh
echo "#Version 0.4 - 20Mar13 meltwater - change to use provided name for home" >> $HDIR/scratchgpio4/scratchgpio4.sh
echo "#Version 1.0 - 29Oct13 sw - change to cd into simplesi_scratch_handler to run servods OK" >> $HDIR/scratchgpio4/scratchgpio4.sh
echo "sudo ps aux | grep 'python.*scratchgpio_handler4.py' | grep -v grep | awk '{print \$2}' | xargs sudo kill -9 " >> $HDIR/scratchgpio4/scratchgpio4.sh
echo "cd $HDIR/scratchgpio4" >> $HDIR/scratchgpio4/scratchgpio4.sh
echo "sudo python scratchgpio_handler4.py &" >> $HDIR/scratchgpio4/scratchgpio4.sh
echo "scratch --document \"$HDIR/Documents/Scratch Projects/rsc.sb\" &" >> $HDIR/scratchgpio4/scratchgpio4.sh

chmod +x $HDIR/scratchgpio4/scratchgpio4.sh
chown -R $USERID:$GROUPID $HDIR/scratchgpio4/scratchgpio4.sh

cp blink11.py $HDIR

cp scratchgpio4.desktop $HDIR/Desktop


mkdir -p $HDIR/Documents
chown -R $USERID:$GROUPID $HDIR/Documents

mkdir -p $HDIR/Documents/Scratch\ Projects
chown -R $USERID:$GROUPID $HDIR/Documents/Scratch\ Projects

cp rsc.sb $HDIR/Documents/Scratch\ Projects
cp GPIOexample.sb $HDIR/Documents/Scratch\ Projects
cp blink11.sb $HDIR/Documents/Scratch\ Projects
chown -R $USERID:$GROUPID $HDIR/Documents/Scratch\ Projects
echo ""
echo "Finished."
