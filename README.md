Thermometer
===========
Thermometer is based on TI's demo "Thermometer" code, which is using sensorTag like hardware produced by TI.

The project is created by IAR 8051 version. Project file is "Thermometer\Projects\ble\Thermometer\CC2541DB\thermometer.eww"

I made a "coin" sized hardware which has a cc2541 and a tmp112. The cc2541 is a ble soc chip and the tmp112 is a temperature sensor.

So, this project communicates with tmp112 by i2c to get the current temperature.

Note, this code is very dirty but can do its work, which has two functions. One is notify realtime temperature, another is sample and store temperature every 5 minutes during one period time.

When powered up, cc2541 will boardcast about 30 seconds. Then it goes into sleep. Press the button once to wake it up and boardcast another 30 seconds. Meanwhile it can be connected. You can get temperature in realtime by writing 0x0018 a '1', with gatttool, that is
    
    char-write-cmd 0x0018 0100
    
if write success, Thermometer will send back temperature every 5 seconds. The value is mutiplied by 100.
Stop sending back temperature can be done by writing 0x0018 a '0'.

Sample and store temperature function can be initialized by writing 0x001b a 'number', which represents the minutes how long the period is( scope is 6 minutes to 600 minutes). And temperature is recorded every 5 minutes.

For example, if you want record temperature for 10 hours. you can just:

    char-write-cmd 0x001b 5802
    
in gatttool. '5802' came from 10 hours, which is 600 minutes, which is 0x258 in hex, which is 5802 because gatttool requires this order.

When you want to see recorded temperature, you can just:
    char-read-hnd 0x0021
The result also requires a swap between high and low bytes in each temperature data.


All the hardware costs about 10 dollars in China. I also open-sourced hardware at:
countemp.strikingly.com . You can make one by yourself or buy one if you like.
