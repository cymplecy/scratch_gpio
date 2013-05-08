#!/bin/bash
# V2 kill any version of the handler running
#sudo ps aux | grep 'python.*scratch_gpio_handler*.py'
#sudo ps aux | grep 'python.*scratch_gpio_handler*.py' | grep -v grep
#sudo ps aux | grep 'python.*scratch_gpio_handler*.py' | grep -v grep | awk '{print $2}'
sudo ps aux | grep 'python.*scratch_gpio_handler2.py' | grep -v grep | awk '{print $2}' | xargs sudo kill -9
sudo ps aux | grep 'python.*scratch_gpio_handler.py' | grep -v grep | awk '{print $2}' | xargs sudo kill -9
#sudo python scratch_gpio_handler.py

