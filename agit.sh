#!/bin/bash
#autobuild new installer
MSG=$1
echo $MSG
echo $HOME
echo
git add .
git commit -am "$MSG"
git push
exit 0
