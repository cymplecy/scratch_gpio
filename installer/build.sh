#!/bin/bash
#copy files to payload folder
<<<<<<< HEAD
SGHVER="5"
echo $SGHVER
echo $HOME
cp $HOME/sghdev/scratch_gpio/Adafruit_I2C.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/Adafruit_PWM_Servo_Driver.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/killsgh.sh $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/scratchgpio_handler${SGHVER}.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_GPIOController.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_PCF8591P.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_PiGlow.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_servod $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_Stepper.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_Adafruit_8x8.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_Adafruit_LEDBackpack.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_PiMatrix.py $HOME/sghdev/scratch_gpio/installer/payload
=======
>>>>>>> V5Dev

cp ../Adafruit_I2C.py ./payload/
cp ../Adafruit_PWM_Servo_Driver.py ./payload/
cp ../killsgh.sh ./payload/
cp ../scratchgpio_handler4.py ./payload/
cp ../sgh_Adafruit_8x8.py ./payload/
cp ../sgh_Adafruit_LEDBackpack.py ./payload/
cp ../sgh_GPIOController.py ./payload/
cp ../sgh_PCF8591P.py ./payload/
cp ../sgh_PiGlow.py ./payload/
cp ../sgh_servod ./payload/
cp ../sgh_Stepper.py ./payload/
cp ../sgh_PiMatrix.py ./payload/



cd payload
tar cf ../payload.tar ./* #tar all the payload files
cd ..

if [ -e "payload.tar" ]; then
    gzip payload.tar #gzip the payload files

    if [ -e "payload.tar.gz" ]; then
        cat decompress.sh payload.tar.gz > install_scratchgpio4.sh # bolt on decompress script
    else
        echo "payload.tar.gz does not exist"
        exit 1
    fi
else
    echo "payload.tar does not exist"
    exit 1
fi
chmod +x install_scratchgpio4.sh #make install script executeable
echo "install_scratchgpio4.sh created"
cp install_scratchgpio4.sh ../ #copy installer to main folder

exit 0
