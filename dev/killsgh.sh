#!/bin/bash
sudo ps aux | grep 'python.*scratch_gpio_handler.py' | grep -v grep | awk '{print $2}' | xargs sudo kill -9
#sudo python scratch_gpio_handler.py

