if [ ! -d ~/piconzero ]; then
  mkdir ~/piconzero
fi
cd ~/piconzero
wget -q http://4tronix.co.uk/piconzero/piconzero.py -O piconzero.py
wget -q http://4tronix.co.uk/piconzero/hcsr04.py -O hcsr04.py
wget -q http://4tronix.co.uk/piconzero/version.py -O version.py
wget -q http://4tronix.co.uk/piconzero/ioTest.py -O ioTest.py
wget -q http://4tronix.co.uk/piconzero/motorTest.py -O motorTest.py
wget -q http://4tronix.co.uk/piconzero/sonarTest.py -O sonarTest.py
wget -q http://4tronix.co.uk/piconzero/tempTest.py -O tempTest.py
wget -q http://4tronix.co.uk/piconzero/10lineTest.py -O 10lineTest.py
wget -q http://4tronix.co.uk/piconzero/pixelTest.py -O pixelTest.py
wget -q http://4tronix.co.uk/piconzero/servoTest.py -O servoTest.py
wget -q http://4tronix.co.uk/piconzero/buttonTest.py -O buttonTest.py

