#!/bin/bash
#copy files to payload folder
#7Feb19 Add in S3GPIOBeta1
#20Aug19 add in S3GPIO Desktop
#21Aug19 reinstate copy s3gpi files to payload

SGHVER=$1
echo $SGHVER
echo $HOME
echo $SGHVER > $HOME/sghdev/scratch_gpio/installer/payload/SGHVER.txt

cp $HOME/sghdev/scratch_gpio/scratchgpio_handler8.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/Adafruit_I2C.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_servod $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/killsgh.sh $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/nunchuck.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/meArm.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/kinematics.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/dht11.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/wstosgh.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/websocket_server.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/sgh_piGPIOExtension.js $HOME/sghdev/scratch_gpio/installer/payload

cp $HOME/sghdev/scratch_gpio/dot.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/btcomm.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/app.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/mock.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/threads.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/utils.py $HOME/sghdev/scratch_gpio/installer/payload

cp $HOME/sghdev/scratch_gpio/sgh_*.py $HOME/sghdev/scratch_gpio/installer/payload

cp $HOME/sghdev/scratch_gpio/S3GPIOServer.py $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/ca.* $HOME/sghdev/scratch_gpio/installer/payload


rm -rf $HOME/sghdev/scratch_gpio/installer/payload/mcpi
mkdir -p $HOME/sghdev/scratch_gpio/installer/payload/mcpi
cp $HOME/sghdev/scratch_gpio/mcpi/* $HOME/sghdev/scratch_gpio/installer/payload/mcpi

rm -rf $HOME/sghdev/scratch_gpio/installer/payload/S3GPIOExtension
mkdir -p $HOME/sghdev/scratch_gpio/installer/payload/S3GPIOExtension
cp $HOME/sghdev/scratch_gpio/S3GPIOExtension/* $HOME/sghdev/scratch_gpio/installer/payload/S3GPIOExtension

cp $HOME/sghdev/scratch_gpio/S3GPIO.desktop $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/s3gpio.sh $HOME/sghdev/scratch_gpio/installer/payload

cp $HOME/sghdev/scratch_gpio/S3GPIOdtop.desktop $HOME/sghdev/scratch_gpio/installer/payload
cp $HOME/sghdev/scratch_gpio/s3gpiodtop.sh $HOME/sghdev/scratch_gpio/installer/payload

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
