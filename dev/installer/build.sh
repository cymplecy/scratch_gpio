#!/bin/bash
cp ../scratchgpio_handler3.py ./payload/ #copy dev version to payload folder
cd payload
tar cf ../payload.tar ./* #tar all the payload files
cd ..

if [ -e "payload.tar" ]; then
    gzip payload.tar #gzip the payload files

    if [ -e "payload.tar.gz" ]; then
        cat decompress.sh payload.tar.gz > install_scratchgpio3.sh # bolt on decompress script
    else
        echo "payload.tar.gz does not exist"
        exit 1
    fi
else
    echo "payload.tar does not exist"
    exit 1
fi
chmod +x install_scratchgpio3.sh #make install script executeable
echo "install_scratchgpio3.sh created"
cp install_scratchgpio3.sh ../ #copy installer to dev folder
#cp ../scratchgpio_handler3.py ../../ # copy handler to base folder
#rm install_scratch_gpio2.sh

exit 0
