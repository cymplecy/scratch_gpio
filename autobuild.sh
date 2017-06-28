#!/bin/bash
#autobuild new installer
SGHVER=$1
echo $SGHVER
echo $HOME
#echo $SGHVER > $HOME/sghdev/scratch_gpio/installer/payload/SGHVER.txt
cd $HOME/sghdev/scratch_gpio/installer
./build.sh $SGHVER
cd  ..
exit 0
