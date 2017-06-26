new (function() {
    var ext = this;
    var fs = require('fs');
    var websocket
    var sensorSplit 
    var sensorDict = {}
    var pinLookup = {'3' : '2', '5' : '3', '7' : '4', '8' : '14', '10' : '15' , '11' : '17', '12' : '18', '13' : '27','15' : '22', '16' : '23', '18' : '24', '19' : '10', '21' : '9', '22' : '25', '23': '11', '24' : '8', '26' : '7', '27' : '0' , '28' : '1', '29' : '5', '31' : '6', '32' : '12', '33' : '13', '35' : '19', '36' : '16', '37' : '26', '38' : '20', '40' : '21'}

    function doConnect() 
    {
      websocket = new WebSocket('ws://localhost:8000/')
      websocket.onopen = function(evt) { onOpen(evt) }
      websocket.onclose = function(evt) { onClose(evt) }
      websocket.onmessage = function(evt) { onMessage(evt) }
      websocket.onerror = function(evt) { onError(evt) }
      console.log('websocket connected from piGPIOExtension')
    }

    function onOpen(evt) {
      console.log('websocket opened')
    }

    function onClose(evt) {
      console.log('websocket closed')
    }

    function onMessage(evt) {
      var data = evt.data
      console.log('msg from sgh:' + data)
      sensorSplit = data.split(":");
      sensorDict[sensorSplit[0]] = sensorSplit[1];
    }

    function onError(evt) {
      var error = evt.data
      console.log('websocket error', error);
      
      websocket.close();
    }

    function sendMessage(message) {
      websocket.send(message);
      console.log('msg to sgh:' + message)
    }

    function doDisconnect() {
      websocket.close();
     }
    
    doConnect();

    // Cleanup function when the extension is unloaded
    ext._shutdown = function ()
    {
        for (pin = 2; pin < 28; pin++)
        {
            if (fs.existsSync("/sys/class/gpio/gpio" + pin))
                fs.writeFileSync("/sys/class/gpio/unexport", pin, "utf8");
        }
    };

    // Status reporting code
    // Use this to report missing hardware, plugin or unsupported browser
    ext._getStatus = function ()
    {
        return {status: 2, msg: 'Ready'};
    };

    ext.set_gpio = function (pin, val) 
    {
        if (pin === '' || pin < 0 || pin > 27) return;

        var dir = 0, lev;
        if (val == 'output high') lev = 1;
        else if (val == 'output low') lev = 0;
        else dir = 1;

		// check the pin is exported
		if (!fs.existsSync("/sys/class/gpio/gpio" + pin)) 

			fs.writeFileSync("/sys/class/gpio/export", pin, "utf8");

		// the ownership of direction takes time to establish, so try this until it succeeds
		while (true)
		{
			try {
				fs.writeFileSync("/sys/class/gpio/gpio" + pin + "/direction", dir == 0 ? "out" : "in", "utf8");
				break;
			}
			catch (error) {
				continue;
			}
		}

		// set the output value
        if (dir == 0)
            sendMessage('pin ' + pin + ' = ' + (lev == 1 ? "1" : "0"));
            fs.writeFileSync("/sys/class/gpio/gpio" + pin + "/value", lev == 1 ? "1" : "0", "utf8");
    };
  
    ext.get_gpio = function (pin) 
    {
        if (pin === '' || pin < 0 || pin > 27) return;

		// check the pin is exported
		if (!fs.existsSync("/sys/class/gpio/gpio" + pin)) 
			fs.writeFileSync("/sys/class/gpio/export", pin);

		// read the pin value
		var data = fs.readFileSync ("/sys/class/gpio/gpio" + pin + "/value", 'utf8');

		if (data.slice(0,1) == "1") return true;
		else return false;
    };
    
//my code
ext.send_broadcast = function (bmsg) 
    {
        sendMessage('broadcast "' + bmsg + '"');

    };
ext.send_joinedBroadcast = function (bmsg1,bmsg2,bmsg3,bmsg4,bmsg5) 
    {
        sendMessage('broadcast "' + bmsg1  + bmsg2  + bmsg3  + bmsg4  + bmsg5 + '"');

    };    
    
ext.send_variable = function (sgh_var,val) 
    {
        sendMessage('sensor-update "' + sgh_var + '" ' + val);

    };    
ext.set_pin = function (pin,val) 
    {
        sendMessage('broadcast "pin' + pin + val +'"');
    };     
ext.get_pin = function (pin) 
    {
        if (pin === '' || pin < 0 || pin > 27) return;

		// check the pin is exported
		if (!fs.existsSync("/sys/class/gpio/gpio" + pin)) 
			fs.writeFileSync("/sys/class/gpio/export", pin);

		// read the pin value
		var data = fs.readFileSync ("/sys/class/gpio/gpio" + pin + "/value", 'utf8');

		if (data.slice(0,1) == "1") return true;
		else return false;
    };    
ext.set_pixel = function (x,y,val) 
    {
        sendMessage('broadcast "pixel' + x + ',' + y + val +'"');
    };      
ext.set_pixels = function (val) 
    {
        sendMessage('broadcast "pixels' + val +'"');
    };    
ext.get_cheerlights = function (val) 
    {
        sendMessage('broadcast "get cheerlights"');
    };     
    

ext.get_sensorMsg = function (sensorName) 
    {
    return sensorDict[sensorName];
    };    
ext.get_cheerlightsSensor = function () 
    {
    return sensorDict['cheerlights'];
    };        
   
          
      
    
    // Block and block menu descriptions
    var descriptor = {
        blocks: [
           [' ', 'set pin %m.pin_numbers to %m.pin_outputs', 'set_pin', '11', 'On'],
            ['b', 'pin %n is high?', 'get_pin', ''],   
            [' ', 'broadcast %s', 'send_broadcast', ' '],
            [' ', 'broadcast %s %s %s %s %s', 'send_joinedBroadcast', ' ', ' ', ' ', ' ', ' '],
            [' ', 'set %s to %s', 'send_variable', '', ''],
            [' ', 'set %m.sgh_variables to %s', 'send_variable', '', '0'],            
            [' ', 'set pixel x:%s y:%s to %s', 'set_pixel', '0', '0', 'red'],   
            [' ', 'set all pixels to %s', 'set_pixels', 'red'], 
            [' ', 'get cheerlight colour', 'get_cheerlights'], 
            [' ', 'set gpio %n to %m.outputs', 'set_gpio', '', 'output high'],
            ['b', 'gpio %n is high?', 'get_gpio', ''],
            ['r', '%s sensor value', 'get_sensorMsg', ''],
            ['r', 'cheerlight colour', 'get_cheerlightsSensor'],            
 
        ],
        menus: {
			outputs: ['output high', 'output low', 'input'],
			pin_outputs: ['On', 'Off', 'High','Low', '1','0'],    
            pin_numbers: ['3', '5', '7', '8', '10' , '11', '12', '13','15', '16', '18', '19', '21', '22', '23', '24', '26', '27', '28', '29', '31', '32', '33', '35', '36', '37', '38', '40'],  
			sgh_variables: ['AddOn', 'MotorA', 'MotorB','Motor1','Motor2'],              
		}
    };

    // Register the extension
    ScratchExtensions.register('Pi GPIO', descriptor, ext);
})();
