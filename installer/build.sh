#!/bin/bash
#copy files to payload folder
SGHVER=$1
echo $SGHVER
echo $PWD
echo $SGHVER > payload/SGHVER.txt

cp ../scratchgpio_handler8.py payload
cp ../Adafruit_I2C.py payload
cp ../sgh_servod payload
cp ../killsgh.sh payload
cp ../nunchuck.py payload
cp ../meArm.py payload
cp ../kinematics.py payload


cp ../sgh_*.py payload

rm -rf payload/mcpi
mkdir -p payload/mcpi
cp ../mcpi/* payload/mcpi

cd payload
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
cp install_scratchgpio${SGHVER}.sh .. #copy installer to main folder
