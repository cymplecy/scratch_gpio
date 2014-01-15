import time
 
sensorids = ["28-000004eb4e02", "28-000004eb6805"]
avgtemperatures = []
for sensor in range(len(sensorids)):
        temperatures = []
        for polltime in range(0,5):
                tfile = open("/sys/bus/w1/devices/"+ sensorids[sensor] +"/w1_slave")
                # Read all of the text in the file.
                text = tfile.read()
                # Close the file now that the text has been read.
                tfile.close()
                # Split the text with new lines (\n) and select the second line.
                secondline = text.split("\n")[1]
                # Split the line into words, referring to the spaces, and select the 10th word (counting from 0).
                temperaturedata = secondline.split(" ")[9]
                # The first two characters are "t=", so get rid of those and convert the temperature from a string to a number.
                temperature = float(temperaturedata[2:])
                # Put the decimal point in the right place and display it.
                temperatures.append(temperature / 1000)
                time.sleep(1)
        temperatures = sorted(temperatures)
        del temperatures[6]
        del temperatures[0]
        avgtemperatures.append(sum(temperatures) / float(len(temperatures)))
