#!/bin/bash
#copy files to payload folder
SGHVER=$1
echo $SGHVER
echo $HOME
echo $SGHVER > $HOME/sghdev/scratch_gpio/installer/payload/SGHVER.txt

cp $HOME/sghdev/scratch_gpio/scratchgpio_handler5.py $HOME/sghdev/scratch_gpio/installer/payload

cp $HOME/sghdev/scratch_gpio/Adafruit_I2C.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/Adafruit_PWM_Servo_Driver.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/killsgh.sh $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_GPIOController.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_PCF8591P.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_PiGlow.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_servod $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_Stepper.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_Adafruit_8x8.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_Adafruit_LEDBackpack.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_PiMatrix.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_*.py $HOME/sghdev/scratch_gpio/installer/payload

cd $HOME/sghdev/scratch_gpio/installer/payload
tar -cf ../payload.tar ./* #tar all the payload files
cd ..

if [ -e "payload.tar" ]; then
    gzip -f payload.tar #gzip the payload files

    if [ -e "payload.tar.gz" ]; then
        cat decompress.sh payload.tar.gz > install_scratchgpio${SGHVER}.sh # bolt on decompress script
    else
        echo "payload.tar.gz does not exist"
        exit 1
    fi
else
    echo "payload.tar does not exist"
    exit 1
fi
chmod +x install_scratchgpio${SGHVER}.sh #make install script executeable
echo "install_scratchgpio"$SGHVER".sh created"
cp install_scratchgpio${SGHVER}.sh $HOME/sghdev/scratch_gpio #copy installer to main folder
cd $HOME/sghdev/scratch_gpio

exit 0
