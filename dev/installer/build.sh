#!/bin/bash
cd payload
tar cf ../payload.tar ./*
cd ..

if [ -e "payload.tar" ]; then
    gzip payload.tar

    if [ -e "payload.tar.gz" ]; then
        cat decompress.sh payload.tar.gz > install_scratch_gpio2.sh
    else
        echo "payload.tar.gz does not exist"
        exit 1
    fi
else
    echo "payload.tar does not exist"
    exit 1
fi
chmod +x install_scratch_gpio2.sh
echo "install_scratch_gpio2.sh created"
exit 0
