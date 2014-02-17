#!/bin/bash
#copy files to payload folder
SGHVER="5"
cp ../Adafruit_I2C.py ./payload/
cp ../Adafruit_PWM_Servo_Driver.py ./payload/
cp ../killsgh.sh ./payload/
cp ../scratchgpio_handler${SGHVER}.py ./payload/
cp ../sgh_GPIOController.py ./payload/
cp ../sgh_PCF8591P.py ./payload/
cp ../sgh_PiGlow.py ./payload/
cp ../sgh_servod ./payload/
cp ../sgh_Stepper.py ./payload/
cp ../sgh_Adafruit_8x8.py ./payload/
cp ../sgh_Adafruit_LEDBackpack.py ./payload/

cd payload
tar cf ../payload.tar ./* #tar all the payload files
cd ..

if [ -e "payload.tar" ]; then
    gzip payload.tar #gzip the payload files

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
cp install_scratchgpio${SGHVER}.sh ../ #copy installer to main folder

exit 0
